#define MIN(X,Y) ((X) < (Y) ? : (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? : (X) : (Y))

#include "ble_client.h"

#include <stdlib.h>
#include <glib.h>
#include <sstream>
#include <unistd.h>

#define SEC_LEVEL "low" // or "medium" or "high"
#define PSM 0
#define MTU 0 //ATT_DEFAULT_LE_MTU

std::map<GIOChannel *, BleClient *> BleClient::ioMap;
GMainLoop *BleClient::event_loop = NULL;
std::vector<void(*)(const uint16_t, const int, const uint8_t*)>
    BleClient::notificationCbFcts;

extern "C" {
GIOChannel *gatt_connect(const char *src, const char *dst,
        const char *dst_type, const char *sec_level,
        int psm, int mtu, BtIOConnect connect_cb,
        GError **gerr);
}

void BleClient::connect_cb(GIOChannel *io, GError *err, gpointer user_data)
{
    gboolean got_error = false;
	uint16_t mtu;
	uint16_t cid;
	GError *gerr = NULL;

	if (err) {
		fprintf(stderr, "%s\n", err->message);
		got_error = TRUE;
	}

	bt_io_get(io, &gerr, BT_IO_OPT_IMTU, &mtu,
				BT_IO_OPT_CID, &cid, BT_IO_OPT_INVALID);

	if (gerr) {
        fprintf(stderr, "connect_cb failed");
		g_error_free(gerr);
		mtu = ATT_DEFAULT_LE_MTU;
	}

	if (cid == ATT_CID)
		mtu = ATT_DEFAULT_LE_MTU;

    ioMap[io]->attrib = g_attrib_new(io, mtu, false);
    ioMap[io]->state = CONNECTED;
    GAttrib *attrib = ioMap[io]->attrib;

    g_idle_add(listen_start, attrib);

    fprintf(stderr, "Connection successful\n");
}

BleClient::BleClient(const std::string &_src,
        const std::string &_dstAddr,
        const std::string &_dstType):
    src(_src),
    dstAddr(_dstAddr),
    dstType(_dstType),
    attrib(NULL),
    iochannel(NULL),
    state(DISCONNECTED)
{
}

bool BleClient::connect()
{
	GError *gerr = NULL;

    if(state != DISCONNECTED)
        return false;

    if(event_loop == NULL)
    {
        int err = pthread_create(&tid, NULL, &startGLoop, NULL);
        if (err != 0)
        {
            fprintf(stderr, "Can't create thread :[%s].\n", strerror(err));
            return false;
        }
    }

    iochannel = gatt_connect(src.c_str(),
            dstAddr.c_str(),
            dstType.c_str(),
            SEC_LEVEL,
            PSM,
            MTU,
            connect_cb,
            &gerr);
    ioMap[iochannel] = this; 
    if(iochannel == NULL)
    {
        fprintf(stderr, "cannot connect.");
        return false;
    }

    state = CONNECTING;
    fprintf(stderr, "Connecting from %s to %s with type %s\n",
            src.c_str(),
            dstAddr.c_str(),
            dstType.c_str());

    g_io_add_watch(iochannel, G_IO_HUP, channel_watcher, NULL);

    return true;
}

void BleClient::disconnect()
{
    if(state == DISCONNECTED)
        return;

    ioMap.erase(iochannel);

	g_attrib_unref(attrib);
	attrib = NULL;

	g_io_channel_shutdown(iochannel, FALSE, NULL);
	g_io_channel_unref(iochannel);
	iochannel = NULL;

    state = DISCONNECTED;

    g_main_loop_quit(event_loop);
    event_loop = NULL;
}

void BleClient::gattWriteCmd(const uint16_t _handle,
        const uint8_t *_data, const int _size)
{
    if(isConnected() == false)
        return;

    gatt_write_cmd(attrib, _handle, _data, _size, NULL, NULL);
}

void BleClient::gattListenToNotification(
        void (*cbFct)(const uint16_t, const int, const uint8_t *))
{
    notificationCbFcts.push_back(cbFct);
}

bool BleClient::isConnected()
{
    if(state == CONNECTED)
        return true;
    return false;
}

void* BleClient::startGLoop(void *_arg)
{
    BleClient::event_loop = g_main_loop_new(NULL, false);
    g_main_loop_run(event_loop);
}

void BleClient::events_handler(const uint8_t *pdu, uint16_t len, gpointer user_data)
{
    GAttrib *attrib = (GAttrib *)user_data;
    uint8_t *opdu;
    uint16_t handle, i, olen = 0;
    size_t plen;

    handle = get_le16(&pdu[1]);

    switch(pdu[0])
    {
        case ATT_OP_HANDLE_NOTIFY:
            //fprintf(stderr, "Notification handle = 0x%04X value: ", handle);
            break;
        default:
            fprintf(stderr, "Invalid opcode.\n");
            return;
    }
    
    int fct_size = notificationCbFcts.size();
    for(int i = 0; i < fct_size; i++)
    {
        void (*fct)(const uint16_t, const int, const uint8_t*);
        fct = notificationCbFcts[i];
        (*fct)(handle, len-3, &pdu[3]);
    }
}

void BleClient::char_read_cb(guint8 status, const guint8 *pdu, guint16 plen,
        gpointer user_data)
{
    struct att_data_list *list;
    uint8_t value[plen];
    ssize_t vlen;
    int i;

    if(status != 0)
    {
        fprintf(stderr, "Characteristics value/descriptor read failed: %s\n",
                att_ecode2str(status));
        return;
    }

    vlen = dec_read_resp(pdu, plen, value, sizeof(value));

    if(vlen < 0)
    {
        fprintf(stderr, "Protocoll error\n");
        return;
    }
}

gboolean BleClient::listen_start(gpointer user_data)
{
    GAttrib *attrib = (GAttrib *)user_data;

    g_attrib_register(attrib, ATT_OP_HANDLE_NOTIFY, GATTRIB_ALL_HANDLES,
            events_handler, attrib, NULL);

    return false;
}

gboolean BleClient::channel_watcher(GIOChannel *io, GIOCondition cond, gpointer user_data)
{
    fprintf(stderr, "channel_watcher called");
    
    BleClient *bleClient = ioMap[io];
    if(bleClient->isConnected() == true)
    {
        fprintf(stderr, "; lost connection, attempting to reconnect.\n");
        bleClient->disconnect();
        usleep(5000000);
        bleClient->connect();
        return false;
    }
    bleClient->disconnect();

    return false;
}
