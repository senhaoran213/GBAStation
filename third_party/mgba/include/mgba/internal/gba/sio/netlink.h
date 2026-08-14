/* SPDX-License-Identifier: MPL-2.0 */
#ifndef GBA_SIO_NETLINK_H
#define GBA_SIO_NETLINK_H

#include <mgba-util/common.h>

CXX_GUARD_START

#include <mgba/core/timing.h>
#include <mgba/internal/gba/sio.h>
#include <mgba-util/socket.h>
#define GBA_NETLINK_FRAME_SIZE 28
#define GBA_NETLINK_TX_CAPACITY 16

enum GBASIONetlinkRole {
	GBA_NETLINK_HOST = 0,
	GBA_NETLINK_JOIN = 1,
};

enum GBASIONetlinkConnectionState {
	GBA_NETLINK_DISCONNECTED = 0,
	GBA_NETLINK_LISTENING,
	GBA_NETLINK_CONNECTING,
	GBA_NETLINK_HANDSHAKE,
	GBA_NETLINK_READY,
	GBA_NETLINK_ERROR,
};

enum GBASIONetlinkTransferState {
	GBA_NETLINK_IDLE = 0,
	GBA_NETLINK_HOST_WAIT_PEER,
	GBA_NETLINK_JOIN_WAIT_CLOCK,
	GBA_NETLINK_TRANSFER,
};

struct GBASIONetlinkTxFrame {
	uint8_t data[GBA_NETLINK_FRAME_SIZE];
	size_t offset;
};

struct GBASIONetlink;

struct GBASIONetlinkRCNT {
	struct GBASIODriver d;
	struct GBASIONetlink* parent;
};

struct GBASIONetlink {
	struct GBASIODriver d;
	struct GBASIONetlinkRCNT rcnt;
	struct mTimingEvent event;
	Socket listener;
	Socket socket;
	struct Address remoteAddress;
	int remotePort;
	enum GBASIONetlinkRole role;
	enum GBASIONetlinkTransferState transferState;
	volatile int connectionState;
	uint32_t session;
	uint32_t sequence;
	uint32_t transferSequence;
	uint16_t received[4];
	uint8_t rx[GBA_NETLINK_FRAME_SIZE];
	size_t rxSize;
	uint8_t pendingStart[GBA_NETLINK_FRAME_SIZE];
	struct GBASIONetlinkTxFrame tx[GBA_NETLINK_TX_CAPACITY];
	size_t txHead;
	size_t txCount;
	uint64_t lastCycle;
	uint64_t aheadHoldAfterCycle;
	uint64_t nextConnectAttempt;
	int32_t linkTime;
	int32_t peerStartTime;
	int32_t transferCycles;
	uint16_t localGp;
	uint16_t peerGp;
	bool transportActive;
	bool connectingSocket;
	bool closing;
	bool transferActive;
	bool remoteDataReady;
	bool localDataSent;
	bool sessionLive;
	bool localMultiActive;
	bool peerMultiActive;
	bool gpPrevSi;
	bool gpPrevSiValid;
	bool gpModeActive;
	bool gpSentInitial;
	bool hasPendingStart;
	char lastError[96];
};

void GBASIONetlinkCreate(struct GBASIONetlink*);
void GBASIONetlinkDestroy(struct GBASIONetlink*);
bool GBASIONetlinkHost(struct GBASIONetlink*, int port);
bool GBASIONetlinkJoin(struct GBASIONetlink*, const char* host, int port);
enum GBASIONetlinkConnectionState GBASIONetlinkGetState(const struct GBASIONetlink*);
const char* GBASIONetlinkGetError(const struct GBASIONetlink*);

CXX_GUARD_END

#endif
