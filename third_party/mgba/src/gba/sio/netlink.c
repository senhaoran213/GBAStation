/* SPDX-License-Identifier: MPL-2.0 */
#include <mgba/internal/gba/sio/netlink.h>

#include <mgba/internal/gba/gba.h>
#include <mgba/internal/gba/io.h>

#define NETLINK_PROTOCOL 6
#define NETLINK_ACTIVE_POLL_CYCLES 64
#define NETLINK_HANDSHAKE_POLL_CYCLES 1024
#define NETLINK_IDLE_POLL_CYCLES (GBA_ARM7TDMI_FREQUENCY / (60 * 64))
#define NETLINK_CONNECT_RETRY_CYCLES (GBA_ARM7TDMI_FREQUENCY / 2)
#define NETLINK_MAX_CLOCK_AHEAD ((int32_t) (3 * (GBA_ARM7TDMI_FREQUENCY / 60)))
#define NETLINK_AHEAD_GRACE_CYCLES (GBA_ARM7TDMI_FREQUENCY / 60)

enum NetlinkFrameType {
	NETLINK_HELLO = 1,
	NETLINK_WELCOME,
	NETLINK_READY_FRAME,
	NETLINK_START,
	NETLINK_DATA,
	NETLINK_GP,
	NETLINK_CLOSE,
	NETLINK_MODE,
};

static bool _init(struct GBASIODriver*);
static void _deinit(struct GBASIODriver*);
static bool _load(struct GBASIODriver*);
static bool _unload(struct GBASIODriver*);
static uint16_t _write(struct GBASIODriver*, uint32_t, uint16_t);
static bool _rcntInit(struct GBASIODriver*);
static void _rcntDeinit(struct GBASIODriver*);
static uint16_t _rcntWrite(struct GBASIODriver*, uint32_t, uint16_t);
static void _process(struct mTiming*, void*, uint32_t);
static void _handle(struct GBASIONetlink*);

static void _put16(uint8_t* p, uint16_t value) {
	p[0] = value >> 8;
	p[1] = value;
}

static void _put32(uint8_t* p, uint32_t value) {
	p[0] = value >> 24;
	p[1] = value >> 16;
	p[2] = value >> 8;
	p[3] = value;
}

static uint16_t _get16(const uint8_t* p) {
	return ((uint16_t) p[0] << 8) | p[1];
}

static uint32_t _get32(const uint8_t* p) {
	return ((uint32_t) p[0] << 24) | ((uint32_t) p[1] << 16) | ((uint32_t) p[2] << 8) | p[3];
}

static enum GBASIONetlinkConnectionState _connectionState(const struct GBASIONetlink* net) {
	int state;
	ATOMIC_LOAD(state, net->connectionState);
	return state;
}

static void _setConnectionState(struct GBASIONetlink* net, enum GBASIONetlinkConnectionState state) {
	ATOMIC_STORE(net->connectionState, state);
}

static const char* _roleName(const struct GBASIONetlink* net) {
	return net->role == GBA_NETLINK_HOST ? "host" : "join";
}

static uint64_t _currentCycle(const struct GBASIONetlink* net) {
	return net->d.p ? mTimingGlobalTime(&net->d.p->p->timing) : net->lastCycle;
}

static bool _isReady(const struct GBASIONetlink* net) {
	return _connectionState(net) == GBA_NETLINK_READY && !net->closing;
}

static bool _isCableReady(const struct GBASIONetlink* net) {
	return _isReady(net) && net->localMultiActive && net->peerMultiActive;
}

static int32_t _transferCyclesFor(uint16_t siocnt) {
	return GBASIOCyclesPerTransfer[GBASIOMultiplayerGetBaud(siocnt)][1];
}

static void _normaliseClock(struct GBASIONetlink* net) {
	if (net->linkTime == INT32_MAX) {
		net->linkTime = 0;
	}
}

static void _advanceLinkTime(struct GBASIONetlink* net, struct mTiming* timing) {
	uint64_t now = mTimingGlobalTime(timing);
	if (!net->lastCycle) {
		net->lastCycle = now;
		return;
	}
	uint64_t elapsed = now - net->lastCycle;
	if (elapsed > INT32_MAX || net->linkTime > INT32_MAX - (int32_t) elapsed) {
		net->linkTime = INT32_MAX;
	} else {
		net->linkTime += elapsed;
	}
	net->lastCycle = now;
}

static void _setError(struct GBASIONetlink* net, const char* reason) {
	strncpy(net->lastError, reason, sizeof(net->lastError) - 1);
	net->lastError[sizeof(net->lastError) - 1] = '\0';
}

static void _close(struct GBASIONetlink* net, const char* reason) {
	if (!net->closing) {
		mLOG(GBA_SIO, WARN, "NETLINK CLOSE role=%s reason=%s", _roleName(net), reason);
		_setError(net, reason);
	}
	net->closing = true;
}

static void _makeFrame(struct GBASIONetlink* net, uint8_t* frame, enum NetlinkFrameType type,
		uint32_t sequence, uint16_t value, uint16_t siocnt) {
	memset(frame, 0, GBA_NETLINK_FRAME_SIZE);
	memcpy(frame, "MGNL", 4);
	frame[4] = NETLINK_PROTOCOL;
	frame[5] = type;
	frame[6] = net->role;
	frame[7] = 2;
	_put32(&frame[8], type == NETLINK_HELLO ? 0 : net->session);
	_put32(&frame[12], sequence);
	_put16(&frame[16], value);
	_put16(&frame[18], siocnt);
	_put32(&frame[20], type == NETLINK_START ? (uint32_t) net->peerStartTime
		: type == NETLINK_WELCOME ? (uint32_t) net->linkTime : 0);
	_put16(&frame[24], net->received[0]);
	_put16(&frame[26], net->received[1]);
}

static bool _queue(struct GBASIONetlink* net, enum NetlinkFrameType type,
		uint32_t sequence, uint16_t value, uint16_t siocnt) {
	if (net->txCount == GBA_NETLINK_TX_CAPACITY) {
		_close(net, "outbound queue full");
		return false;
	}
	size_t tail = (net->txHead + net->txCount) % GBA_NETLINK_TX_CAPACITY;
	_makeFrame(net, net->tx[tail].data, type, sequence, value, siocnt);
	net->tx[tail].offset = 0;
	++net->txCount;
	return true;
}

static void _publishMultiMode(struct GBASIONetlink* net) {
	if (!_isReady(net)) {
		return;
	}
	uint16_t siocnt = net->d.p ? net->d.p->siocnt : 0;
	_queue(net, NETLINK_MODE, net->transferSequence, net->localMultiActive ? 1 : 0, siocnt);
}

static void _flush(struct GBASIONetlink* net) {
	while (!SOCKET_FAILED(net->socket) && net->txCount && !net->closing) {
		struct GBASIONetlinkTxFrame* frame = &net->tx[net->txHead];
		ssize_t sent = SocketSend(net->socket, frame->data + frame->offset,
			GBA_NETLINK_FRAME_SIZE - frame->offset);
		if (sent > 0) {
			frame->offset += sent;
			if (frame->offset == GBA_NETLINK_FRAME_SIZE) {
				frame->offset = 0;
				net->txHead = (net->txHead + 1) % GBA_NETLINK_TX_CAPACITY;
				--net->txCount;
			}
			continue;
		}
		if (!sent) {
			_close(net, "send returned zero");
		} else if (!SocketWouldBlock()) {
			_close(net, "send failed");
		}
		break;
	}
}

static void _sendCloseBestEffort(struct GBASIONetlink* net) {
	if (SOCKET_FAILED(net->socket) || !net->session) {
		return;
	}
	uint8_t frame[GBA_NETLINK_FRAME_SIZE];
	_makeFrame(net, frame, NETLINK_CLOSE, net->transferSequence, 0, 0);
	SocketSend(net->socket, frame, sizeof(frame));
}

static void _setCableRegisters(struct GBASIONetlink* net, bool ready) {
	if (!net->d.p) {
		return;
	}
	struct GBASIO* sio = net->d.p;
	if (sio->mode != SIO_MULTI) {
		return;
	}
	// Model the cable line phases: SC is low during a transfer and high
	// while idle; SD identifies the slave. These four
	// RCNT data bits are hardware status in multiplayer mode.
	uint16_t pins;
	if (net->role == GBA_NETLINK_JOIN) {
		pins = net->transferActive ? 6 : 7;
	} else {
		pins = net->transferActive ? 2 : 3;
	}
	sio->rcnt = (sio->rcnt & ~0xF) | pins;
	if (net->role == GBA_NETLINK_JOIN) {
		sio->siocnt = GBASIOMultiplayerFillSlave(sio->siocnt);
	} else {
		sio->siocnt = GBASIOMultiplayerClearSlave(sio->siocnt);
	}
	sio->siocnt = GBASIOMultiplayerSetReady(sio->siocnt, ready);
	if (ready) {
		sio->siocnt = GBASIOMultiplayerClearError(sio->siocnt);
	}
}

static void _cancelTransfer(struct GBASIONetlink* net, bool error) {
	if (net->hasPendingStart) {
		uint32_t pendingSequence = _get32(&net->pendingStart[12]);
		if (pendingSequence > net->transferSequence) {
			net->transferSequence = pendingSequence;
		}
	}
	net->transferActive = false;
	net->remoteDataReady = false;
	net->localDataSent = false;
	net->hasPendingStart = false;
	net->transferState = GBA_NETLINK_IDLE;
	if (net->d.p) {
		net->d.p->siocnt = GBASIOMultiplayerClearBusy(net->d.p->siocnt);
		_setCableRegisters(net, _isCableReady(net));
		if (error) {
			net->d.p->siocnt = GBASIOMultiplayerFillError(net->d.p->siocnt);
		}
	}
}

static void _resetCableEpoch(struct GBASIONetlink* net) {
	_cancelTransfer(net, false);
	net->sessionLive = false;
	net->linkTime = 0;
	net->peerStartTime = 0;
	net->transferCycles = 0;
	net->aheadHoldAfterCycle = 0;
	net->received[0] = 0xFFFF;
	net->received[1] = 0xFFFF;
	net->received[2] = 0xFFFF;
	net->received[3] = 0xFFFF;
}

static void _resetPeerSession(struct GBASIONetlink* net) {
	net->peerMultiActive = false;
	_cancelTransfer(net, false);
	net->session = 0;
	net->sequence = 0;
	net->transferSequence = 0;
	net->linkTime = 0;
	net->lastCycle = 0;
	net->aheadHoldAfterCycle = 0;
	net->rxSize = 0;
	net->txHead = 0;
	net->txCount = 0;
	net->sessionLive = false;
	net->peerGp = 0;
	net->gpPrevSiValid = false;
	net->gpSentInitial = false;
}

static uint32_t _newSession(struct GBASIONetlink* net) {
	uint64_t seed = (uintptr_t) net;
	if (net->d.p) {
		seed ^= mTimingGlobalTime(&net->d.p->p->timing);
	}
	seed ^= seed >> 33;
	seed *= UINT64_C(0xff51afd7ed558ccd);
	seed ^= seed >> 33;
	uint32_t session = seed ^ (seed >> 32);
	return session ? session : 1;
}

static uint16_t _mergeGp(struct GBASIONetlink* net, uint16_t local) {
	if ((local >> 14) != 2) {
		return local;
	}
	static const uint8_t peerPin[4] = { 0, 1, 3, 2 };
	uint16_t merged = local;
	for (int pin = 0; pin < 4; ++pin) {
		if (local & (1 << (pin + 4))) {
			continue;
		}
		int level = 1;
		if ((net->peerGp >> 14) == 2) {
			int source = peerPin[pin];
			if (net->peerGp & (1 << (source + 4))) {
				level = (net->peerGp >> source) & 1;
			}
		}
		merged = (merged & ~(1 << pin)) | (level << pin);
	}
	return merged;
}

static void _commitGp(struct GBASIONetlink* net, bool allowIrq) {
	if (!net->d.p || !net->gpModeActive) {
		return;
	}
	uint16_t merged = _mergeGp(net, net->localGp);
	net->d.p->rcnt = merged;
	bool si = (merged >> 2) & 1;
	if (allowIrq && (merged & 0x0100) && !(merged & 0x0040)
			&& net->gpPrevSiValid && net->gpPrevSi && !si) {
		GBARaiseIRQ(net->d.p->p, GBA_IRQ_SIO, 0);
	}
	net->gpPrevSi = si;
	net->gpPrevSiValid = true;
}

static void _finish(struct GBASIONetlink* net) {
	struct GBASIO* sio = net->d.p;
	sio->p->memory.io[REG_SIOMULTI0 >> 1] = net->received[0];
	sio->p->memory.io[REG_SIOMULTI1 >> 1] = net->received[1];
	sio->p->memory.io[REG_SIOMULTI2 >> 1] = 0xFFFF;
	sio->p->memory.io[REG_SIOMULTI3 >> 1] = 0xFFFF;
	sio->rcnt |= 1;
	sio->siocnt = GBASIOMultiplayerClearBusy(sio->siocnt);
	sio->siocnt = GBASIOMultiplayerSetId(sio->siocnt,
		net->role == GBA_NETLINK_HOST ? 0 : 1);
	net->transferActive = false;
	net->transferState = GBA_NETLINK_IDLE;
	net->remoteDataReady = false;
	net->localDataSent = false;
	net->sessionLive = _isCableReady(net);
	// A game may stop the master's transfer stream and then need a short
	// amount of CPU time to clear SIOCNT IRQ or change serial mode. Entering
	// the socket hold on the very next timing callback advances mGBA only one
	// emulated cycle per 1 ms wait, turning roughly 10,000 teardown cycles in
	// Pokemon into several seconds of apparent black-screen delay. One frame
	// is still well inside the game's quiet-vblank tolerance while giving its
	// teardown path time to declare that the stream has ended.
	net->aheadHoldAfterCycle = net->sessionLive
		? _currentCycle(net) + NETLINK_AHEAD_GRACE_CYCLES : 0;
	_setCableRegisters(net, _isCableReady(net));
	if (GBASIOMultiplayerIsIrq(sio->siocnt)) {
		GBARaiseIRQ(sio->p, GBA_IRQ_SIO, 0);
	}
}

static void _startJoinTransfer(struct GBASIONetlink* net, const uint8_t* frame) {
	uint32_t sequence = _get32(&frame[12]);
	if (sequence <= net->transferSequence) {
		return;
	}
	if (sequence != net->transferSequence + 1) {
		_close(net, "future start sequence");
		return;
	}
	if (!net->d.p || net->d.p->mode != SIO_MULTI) {
		// The slave has already left this game-level MULTI epoch. A real
		// master's clock still completes with an absent slave slot (FFFF), so
		// consume the sequence and return terminal DATA without re-entering
		// MULTI or raising a local IRQ. Silently dropping this START leaves the
		// host frozen forever at its hardware transfer boundary.
		net->transferSequence = sequence;
		_queue(net, NETLINK_DATA, sequence, 0xFFFF, net->d.p ? net->d.p->siocnt : 0);
		return;
	}
	if (net->transferActive) {
		if (net->hasPendingStart) {
			_close(net, "multiple pending starts");
			return;
		}
		memcpy(net->pendingStart, frame, GBA_NETLINK_FRAME_SIZE);
		net->hasPendingStart = true;
		return;
	}
	net->transferSequence = sequence;
	net->transferActive = true;
	net->transferState = GBA_NETLINK_JOIN_WAIT_CLOCK;
	net->localDataSent = false;
	net->peerStartTime = _get32(&frame[20]);
	net->transferCycles = _transferCyclesFor(_get16(&frame[18]));
	net->received[0] = _get16(&frame[16]);
	net->received[1] = 0xFFFF;
	// The master's word is visible in SIOMULTI0 as soon as the serial clock
	// reaches the slave. Do not defer every receive register until completion:
	// games may poll SIOMULTI while Busy is still set.
	net->d.p->p->memory.io[REG_SIOMULTI0 >> 1] = net->received[0];
	net->d.p->p->memory.io[REG_SIOMULTI1 >> 1] = 0xFFFF;
	net->d.p->p->memory.io[REG_SIOMULTI2 >> 1] = 0xFFFF;
	net->d.p->p->memory.io[REG_SIOMULTI3 >> 1] = 0xFFFF;
	net->d.p->siocnt = GBASIOMultiplayerFillBusy(net->d.p->siocnt);
	_setCableRegisters(net, true);
}

static void _handlePeerMode(struct GBASIONetlink* net, const uint8_t* frame) {
	uint16_t value = _get16(&frame[16]);
	if (value > 1) {
		_close(net, "invalid multi mode state");
		return;
	}
	bool active = value != 0;
	bool changed = net->peerMultiActive != active;
	net->peerMultiActive = active;

	if (!active) {
		// Keep an already clocked Host transfer alive long enough to receive
		// the terminal FFFF DATA sent for the crossed START. With no transfer
		// in flight, fully return the game-side cable state to its pre-room
		// epoch while preserving the TCP session.
		if (!net->transferActive) {
			_resetCableEpoch(net);
		} else {
			net->sessionLive = false;
			net->aheadHoldAfterCycle = 0;
		}
		_setCableRegisters(net, false);
		return;
	}

	if (changed && net->localMultiActive) {
		_resetCableEpoch(net);
		net->lastCycle = _currentCycle(net);
		// Reply once on a false->true transition. This makes re-entry a fresh
		// two-sided epoch even if one game reached MULTI before the other.
		_publishMultiMode(net);
	}
	_setCableRegisters(net, _isCableReady(net));
}

static void _handle(struct GBASIONetlink* net) {
	uint8_t* frame = net->rx;
	uint8_t type = frame[5];
	uint8_t peerRole = frame[6];
	uint32_t session = _get32(&frame[8]);
	uint32_t sequence = _get32(&frame[12]);
	if (memcmp(frame, "MGNL", 4) || frame[4] != NETLINK_PROTOCOL) {
		_close(net, "protocol mismatch");
		return;
	}
	if (frame[7] != 2 || peerRole == net->role || peerRole > GBA_NETLINK_JOIN) {
		_close(net, "peer role mismatch");
		return;
	}
	if (type == NETLINK_HELLO) {
		if (net->role != GBA_NETLINK_HOST || session || _connectionState(net) != GBA_NETLINK_HANDSHAKE) {
			_close(net, "unexpected hello");
			return;
		}
		_normaliseClock(net);
		net->session = _newSession(net);
		_queue(net, NETLINK_WELCOME, 0, 0, 0);
		mLOG(GBA_SIO, INFO, "NETLINK HANDSHAKE role=host session=%08X", net->session);
		return;
	}
	if (type == NETLINK_WELCOME) {
		if (net->role != GBA_NETLINK_JOIN || !session || _connectionState(net) != GBA_NETLINK_HANDSHAKE) {
			_close(net, "unexpected welcome");
			return;
		}
		net->session = session;
		net->linkTime = _get32(&frame[20]);
		_queue(net, NETLINK_READY_FRAME, 0, 0, 0);
		_setConnectionState(net, GBA_NETLINK_READY);
		_setCableRegisters(net, _isCableReady(net));
		_publishMultiMode(net);
		mLOG(GBA_SIO, INFO, "NETLINK CONNECTED role=join session=%08X", net->session);
		return;
	}
	if (type == NETLINK_READY_FRAME) {
		if (net->role != GBA_NETLINK_HOST || !net->session || session != net->session
				|| _connectionState(net) != GBA_NETLINK_HANDSHAKE) {
			_close(net, "unexpected ready");
			return;
		}
		_setConnectionState(net, GBA_NETLINK_READY);
		_setCableRegisters(net, _isCableReady(net));
		_publishMultiMode(net);
		mLOG(GBA_SIO, INFO, "NETLINK CONNECTED role=host session=%08X", net->session);
		return;
	}
	if (!_isReady(net) || !net->session || session != net->session) {
		_close(net, "session mismatch");
		return;
	}
	switch (type) {
	case NETLINK_START:
		if (net->role != GBA_NETLINK_JOIN) {
			_close(net, "start sent to host");
			return;
		}
		_startJoinTransfer(net, frame);
		break;
	case NETLINK_DATA:
		if (net->role == GBA_NETLINK_HOST && !net->transferActive
				&& sequence <= net->transferSequence) {
			break;
		}
		if (net->role != GBA_NETLINK_HOST || !net->transferActive
				|| sequence != net->transferSequence || net->remoteDataReady) {
			_close(net, "unexpected data sequence");
			return;
		}
		net->received[1] = _get16(&frame[16]);
		net->remoteDataReady = true;
		break;
	case NETLINK_GP:
		net->peerGp = _get16(&frame[16]);
		_commitGp(net, true);
		break;
	case NETLINK_MODE:
		_handlePeerMode(net, frame);
		break;
	case NETLINK_CLOSE:
		_close(net, "peer requested close");
		break;
	default:
		_close(net, "unexpected frame type");
		break;
	}
}

static void _receive(struct GBASIONetlink* net) {
	while (!SOCKET_FAILED(net->socket) && !net->closing) {
		ssize_t got = SocketRecv(net->socket, net->rx + net->rxSize,
			GBA_NETLINK_FRAME_SIZE - net->rxSize);
		if (got > 0) {
			net->rxSize += got;
			if (net->rxSize == GBA_NETLINK_FRAME_SIZE) {
				_handle(net);
				net->rxSize = 0;
			}
			continue;
		}
		if (!got) {
			_close(net, "peer disconnected");
		} else if (!SocketWouldBlock()) {
			_close(net, "receive failed");
		}
		break;
	}
}

static void _tcpConnected(struct GBASIONetlink* net) {
	net->connectingSocket = false;
	SocketSetBlocking(net->socket, false);
	SocketSetTCPPush(net->socket, true);
	_setConnectionState(net, GBA_NETLINK_HANDSHAKE);
	if (net->role == GBA_NETLINK_JOIN) {
		_queue(net, NETLINK_HELLO, 0, 0, 0);
	}
	mLOG(GBA_SIO, INFO, "NETLINK TCP_CONNECTED role=%s", _roleName(net));
}

static void _beginConnect(struct GBASIONetlink* net, uint64_t now) {
	bool connected = false;
	net->socket = SocketConnectTCPNonBlocking(net->remotePort, &net->remoteAddress, &connected);
	if (SOCKET_FAILED(net->socket)) {
		net->nextConnectAttempt = now + NETLINK_CONNECT_RETRY_CYCLES;
		return;
	}
	net->connectingSocket = !connected;
	if (connected) {
		_tcpConnected(net);
	}
}

static void _pollConnect(struct GBASIONetlink* net, uint64_t now) {
	if (SOCKET_FAILED(net->socket)) {
		if (now >= net->nextConnectAttempt) {
			_beginConnect(net, now);
		}
		return;
	}
	if (!net->connectingSocket) {
		return;
	}
	Socket writable[1] = { net->socket };
	Socket errors[1] = { net->socket };
	if (SocketPoll(1, NULL, writable, errors, 0) <= 0) {
		return;
	}
	int error = SocketGetConnectionError(net->socket);
	if (!error) {
		_tcpConnected(net);
		return;
	}
	SocketClose(net->socket);
	net->socket = INVALID_SOCKET;
	net->connectingSocket = false;
	net->nextConnectAttempt = now + NETLINK_CONNECT_RETRY_CYCLES;
}

static void _accept(struct GBASIONetlink* net) {
	if (net->role != GBA_NETLINK_HOST || SOCKET_FAILED(net->listener)
			|| !SOCKET_FAILED(net->socket) || _connectionState(net) != GBA_NETLINK_LISTENING) {
		return;
	}
	Socket accepted = SocketAccept(net->listener, NULL);
	if (SOCKET_FAILED(accepted)) {
		return;
	}
	_resetPeerSession(net);
	net->socket = accepted;
	_tcpConnected(net);
}

static bool _waitForSocket(struct GBASIONetlink* net, int timeoutMs) {
	if (SOCKET_FAILED(net->socket)) {
		return false;
	}
	Socket readable[1] = { net->socket };
	int result = SocketPoll(1, readable, NULL, NULL, timeoutMs);
	if (result > 0) {
		_receive(net);
		return true;
	}
	return false;
}

void GBASIONetlinkCreate(struct GBASIONetlink* net) {
	memset(net, 0, sizeof(*net));
	net->listener = INVALID_SOCKET;
	net->socket = INVALID_SOCKET;
	net->d.init = _init;
	net->d.deinit = _deinit;
	net->d.load = _load;
	net->d.unload = _unload;
	net->d.writeRegister = _write;
	net->rcnt.parent = net;
	net->rcnt.d.init = _rcntInit;
	net->rcnt.d.deinit = _rcntDeinit;
	net->rcnt.d.writeRegister = _rcntWrite;
	net->event.context = net;
	net->event.name = "GBA SIO Netlink";
	net->event.callback = _process;
	net->event.priority = 0x80;
	_setConnectionState(net, GBA_NETLINK_DISCONNECTED);
}

void GBASIONetlinkDestroy(struct GBASIONetlink* net) {
	_sendCloseBestEffort(net);
	if (net->d.p && mTimingIsScheduled(&net->d.p->p->timing, &net->event)) {
		mTimingDeschedule(&net->d.p->p->timing, &net->event);
	}
	if (!SOCKET_FAILED(net->socket)) {
		SocketClose(net->socket);
	}
	if (!SOCKET_FAILED(net->listener)) {
		SocketClose(net->listener);
	}
	net->socket = INVALID_SOCKET;
	net->listener = INVALID_SOCKET;
	net->transportActive = false;
	_setConnectionState(net, GBA_NETLINK_DISCONNECTED);
}

bool GBASIONetlinkHost(struct GBASIONetlink* net, int port) {
	net->role = GBA_NETLINK_HOST;
	net->listener = SocketOpenTCP(port, NULL);
	if (SOCKET_FAILED(net->listener) || SocketListen(net->listener, 1) < 0) {
		_setError(net, "could not listen on TCP port");
		GBASIONetlinkDestroy(net);
		_setConnectionState(net, GBA_NETLINK_ERROR);
		return false;
	}
	SocketSetBlocking(net->listener, false);
	_setConnectionState(net, GBA_NETLINK_LISTENING);
	return true;
}

bool GBASIONetlinkJoin(struct GBASIONetlink* net, const char* host, int port) {
	net->role = GBA_NETLINK_JOIN;
	if (SocketResolveHost(host, &net->remoteAddress)) {
		_setError(net, "could not resolve host");
		_setConnectionState(net, GBA_NETLINK_ERROR);
		return false;
	}
	net->remotePort = port;
	net->nextConnectAttempt = 0;
	_setConnectionState(net, GBA_NETLINK_CONNECTING);
	return true;
}

enum GBASIONetlinkConnectionState GBASIONetlinkGetState(const struct GBASIONetlink* net) {
	return _connectionState(net);
}

const char* GBASIONetlinkGetError(const struct GBASIONetlink* net) {
	return net->lastError;
}

static bool _init(struct GBASIODriver* driver) {
	struct GBASIONetlink* net = (struct GBASIONetlink*) driver;
	net->transportActive = true;
	net->lastCycle = mTimingGlobalTime(&driver->p->p->timing);
	_setCableRegisters(net, _isCableReady(net));
	if (!mTimingIsScheduled(&driver->p->p->timing, &net->event)) {
		mTimingSchedule(&driver->p->p->timing, &net->event, 0);
	}
	return true;
}

static void _deinit(struct GBASIODriver* driver) {
	struct GBASIONetlink* net = (struct GBASIONetlink*) driver;
	net->transportActive = false;
	if (mTimingIsScheduled(&driver->p->p->timing, &net->event)) {
		mTimingDeschedule(&driver->p->p->timing, &net->event);
	}
}

static bool _load(struct GBASIODriver* driver) {
	struct GBASIONetlink* net = (struct GBASIONetlink*) driver;
	// Returning to MULTI begins a new cable-clock epoch. Time spent in GP,
	// Normal or UART mode must not count as silence in the old transfer
	// stream or immediately trigger the join-side ahead hold.
	net->localMultiActive = true;
	_resetCableEpoch(net);
	net->lastCycle = mTimingGlobalTime(&driver->p->p->timing);
	_setCableRegisters(net, _isCableReady(net));
	_publishMultiMode(net);
	return true;
}

static bool _unload(struct GBASIODriver* driver) {
	struct GBASIONetlink* net = (struct GBASIONetlink*) driver;
	// Leaving MULTI ends the current transfer stream. The TCP session stays
	// connected for a later return, but no SIO pacing wait may survive into
	// another hardware mode (Pokémon leaves MULTI while exiting the room).
	net->localMultiActive = false;
	// Re-entering MULTI must require a fresh active announcement from the
	// peer, not the remembered state from the room that just ended.
	net->peerMultiActive = false;
	_resetCableEpoch(net);
	net->lastCycle = mTimingGlobalTime(&driver->p->p->timing);
	_publishMultiMode(net);
	return true;
}

static bool _rcntInit(struct GBASIODriver* driver) {
	UNUSED(driver);
	return true;
}

static void _rcntDeinit(struct GBASIODriver* driver) {
	UNUSED(driver);
}

static uint16_t _rcntWrite(struct GBASIODriver* driver, uint32_t address, uint16_t value) {
	struct GBASIONetlinkRCNT* observer = (struct GBASIONetlinkRCNT*) driver;
	struct GBASIONetlink* net = observer->parent;
	if (address != REG_RCNT) {
		return value;
	}
	bool wasGp = net->gpModeActive;
	bool nowGp = (value >> 14) == 2;
	uint16_t committed = net->d.p ? net->d.p->rcnt : value;
	if (nowGp) {
		// In GP mode the low nibble is the CPU's output latch/input request;
		// the generic mGBA RCNT path deliberately leaves it to the driver.
		committed = value & 0xC1FF;
		if (net->d.p) {
			net->d.p->rcnt = committed;
		}
	}
	uint16_t previous = net->localGp;
	net->localGp = committed;
	net->gpModeActive = nowGp;
	if (nowGp && !wasGp) {
		net->gpPrevSiValid = false;
		_cancelTransfer(net, false);
	}
	if (_isReady(net) && (!net->gpSentInitial || previous != committed)) {
		_queue(net, NETLINK_GP, 0, committed, 0);
		net->gpSentInitial = true;
	}
	if (nowGp) {
		_commitGp(net, false);
	}
	return value;
}

static uint16_t _write(struct GBASIODriver* driver, uint32_t address, uint16_t value) {
	struct GBASIONetlink* net = (struct GBASIONetlink*) driver;
	if (address != REG_SIOCNT) {
		return value;
	}
	_setCableRegisters(net, _isCableReady(net));
	if ((value & 0x0080) && net->role == GBA_NETLINK_HOST
			&& _isCableReady(net) && !net->transferActive) {
		_normaliseClock(net);
		net->transferActive = true;
		net->transferState = GBA_NETLINK_HOST_WAIT_PEER;
		net->remoteDataReady = false;
		net->localDataSent = true;
		net->transferSequence = ++net->sequence;
		net->received[0] = net->d.p->p->memory.io[REG_SIOMLT_SEND >> 1];
		net->received[1] = 0xFFFF;
		// Match cable hardware visibility during an active transfer:
		// publish the master's slot immediately and leave absent/unreceived
		// slots high until the transfer commits.
		net->d.p->p->memory.io[REG_SIOMULTI0 >> 1] = net->received[0];
		net->d.p->p->memory.io[REG_SIOMULTI1 >> 1] = 0xFFFF;
		net->d.p->p->memory.io[REG_SIOMULTI2 >> 1] = 0xFFFF;
		net->d.p->p->memory.io[REG_SIOMULTI3 >> 1] = 0xFFFF;
		net->peerStartTime = net->linkTime;
		net->transferCycles = _transferCyclesFor(value);
		net->linkTime = 0;
		// A transfer may start between transport timing callbacks. Reset the
		// accumulator origin together with linkTime so the next callback does
		// not count pre-START idle cycles toward the hardware transfer time.
		net->lastCycle = mTimingGlobalTime(&net->d.p->p->timing);
		net->d.p->siocnt = GBASIOMultiplayerFillBusy(net->d.p->siocnt);
		_setCableRegisters(net, true);
		_queue(net, NETLINK_START, net->transferSequence, net->received[0], value);
	}
	value &= 0xFF83;
	value |= net->d.p->siocnt & 0x00FC;
	return value;
}

static void _serviceTransfer(struct GBASIONetlink* net) {
	if (net->role == GBA_NETLINK_JOIN && !net->transferActive && net->hasPendingStart
			&& net->d.p->mode == SIO_MULTI) {
		uint8_t frame[GBA_NETLINK_FRAME_SIZE];
		memcpy(frame, net->pendingStart, sizeof(frame));
		net->hasPendingStart = false;
		_startJoinTransfer(net, frame);
	}
	if (net->role == GBA_NETLINK_JOIN && net->transferActive && !net->localDataSent) {
		int32_t clockLag = net->peerStartTime - net->linkTime;
		if (clockLag > net->transferCycles) {
			net->linkTime = net->peerStartTime;
		}
		if (net->linkTime >= net->peerStartTime) {
			net->received[1] = net->d.p->p->memory.io[REG_SIOMLT_SEND >> 1];
			// Preserve time the join side has already run past the host's
			// published start boundary. Clearing the accumulator here loses
			// that overshoot on every packet, so the join core can gain many
			// frames while the between-transfer ahead limiter never sees it.
			net->linkTime -= net->peerStartTime;
			net->localDataSent = true;
			net->transferState = GBA_NETLINK_TRANSFER;
			_queue(net, NETLINK_DATA, net->transferSequence, net->received[1], net->d.p->siocnt);
		}
	}
	if (net->role == GBA_NETLINK_JOIN && net->transferActive && net->localDataSent
			&& net->linkTime >= net->transferCycles) {
		net->linkTime -= net->transferCycles;
		_finish(net);
	}
	if (net->role == GBA_NETLINK_HOST && net->transferActive && net->remoteDataReady
			&& net->linkTime >= net->transferCycles) {
		net->linkTime -= net->transferCycles;
		_finish(net);
	}
}

static void _serviceClosing(struct GBASIONetlink* net, struct mTiming* timing) {
	if (!SOCKET_FAILED(net->socket)) {
		SocketClose(net->socket);
		net->socket = INVALID_SOCKET;
	}
	_cancelTransfer(net, true);
	net->rxSize = 0;
	net->txHead = 0;
	net->txCount = 0;
	net->session = 0;
	net->sessionLive = false;
	net->peerGp = 0;
	net->gpSentInitial = false;
	_setCableRegisters(net, false);
	if (net->role == GBA_NETLINK_HOST && !SOCKET_FAILED(net->listener)) {
		net->closing = false;
		_setConnectionState(net, GBA_NETLINK_LISTENING);
		mLOG(GBA_SIO, INFO, "NETLINK LISTENING role=host after peer disconnect");
		if (net->transportActive) {
			mTimingSchedule(timing, &net->event, NETLINK_IDLE_POLL_CYCLES);
		}
	} else {
		_setConnectionState(net, GBA_NETLINK_ERROR);
	}
}

static void _process(struct mTiming* timing, void* user, uint32_t cyclesLate) {
	UNUSED(cyclesLate);
	struct GBASIONetlink* net = user;
	_advanceLinkTime(net, timing);
	if (net->closing) {
		_serviceClosing(net, timing);
		return;
	}

	enum GBASIONetlinkConnectionState connection = _connectionState(net);
	if (connection == GBA_NETLINK_LISTENING) {
		_accept(net);
	} else if (connection == GBA_NETLINK_CONNECTING) {
		_pollConnect(net, mTimingGlobalTime(timing));
	}
	if (!SOCKET_FAILED(net->socket) && !net->connectingSocket) {
		_flush(net);
		_receive(net);
		_flush(net);
	}
	if (net->closing) {
		_serviceClosing(net, timing);
		return;
	}

	if (_isReady(net)) {
		if (!net->gpSentInitial) {
			net->localGp = net->d.p ? net->d.p->rcnt : 0;
			_queue(net, NETLINK_GP, 0, net->localGp, 0);
			net->gpSentInitial = true;
		}
		_commitGp(net, true);
		_serviceTransfer(net);
		_flush(net);
	}
	if (net->closing) {
		_serviceClosing(net, timing);
		return;
	}

	bool waitAtBoundary = net->role == GBA_NETLINK_HOST && net->transferActive
		&& net->linkTime >= net->transferCycles && !net->remoteDataReady;
	bool holdAhead = net->role == GBA_NETLINK_JOIN && net->d.p && net->d.p->mode == SIO_MULTI
		// Pokémon clears the serial IRQ enable while tearing down the Cable
		// Club stream (SIOCNT 0x2000/0x201C), before it switches out of MULTI.
		// Holding in that window prevents the game from ever executing the
		// mode switch and leaves the join screen black until Disconnect.
		&& GBASIOMultiplayerIsIrq(net->d.p->siocnt)
		&& !net->transferActive && !net->hasPendingStart && net->sessionLive
		&& net->lastCycle >= net->aheadHoldAfterCycle
		&& net->linkTime > NETLINK_MAX_CLOCK_AHEAD;
	if ((waitAtBoundary || holdAhead) && _isReady(net)) {
		_waitForSocket(net, 1);
		_flush(net);
		if (net->transportActive && !net->closing) {
			mTimingSchedule(timing, &net->event, 1);
		}
		return;
	}

	if (!net->transportActive) {
		return;
	}
	int32_t next = NETLINK_IDLE_POLL_CYCLES;
	connection = _connectionState(net);
	if (net->transferActive || net->hasPendingStart) {
		next = NETLINK_ACTIVE_POLL_CYCLES;
	} else if (connection == GBA_NETLINK_CONNECTING || connection == GBA_NETLINK_HANDSHAKE) {
		next = NETLINK_HANDSHAKE_POLL_CYCLES;
	}
	mTimingSchedule(timing, &net->event, next);
}
