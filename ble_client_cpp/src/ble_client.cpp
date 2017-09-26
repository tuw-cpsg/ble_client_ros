#define MIN(X,Y) ((X) < (Y) ? : (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? : (X) : (Y))

#include "ble_client.h"

#include <stdlib.h>
#include <glib.h>
#include <sstream>

#define SEC_LEVEL "low" // or "medium" or "high"
#define PSM 0
#define MTU 0 //ATT_DEFAULT_LE_MTU

std::map<GIOChannel *, BleClient *> BleClient::ioMap;
GMainLoop *BleClient::event_loop = NULL;
std::map<uint16_t, std::vector<void(*)(int, const uint8_t*)>> BleClient::handleToCbFct;

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
		printf("%s\n", err->message);
		got_error = TRUE;
	}

	bt_io_get(io, &gerr, BT_IO_OPT_IMTU, &mtu,
				BT_IO_OPT_CID, &cid, BT_IO_OPT_INVALID);

	if (gerr) {
		g_error_free(gerr);
		mtu = ATT_DEFAULT_LE_MTU;
	}

	if (cid == ATT_CID)
		mtu = ATT_DEFAULT_LE_MTU;

    ioMap[io]->attrib = g_attrib_new(io, mtu, false);
    ioMap[io]->state = CONNECTED;
    GAttrib *attrib = ioMap[io]->attrib;

    g_idle_add(listen_start, attrib);

    printf("Connection successful\n");
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
    if(event_loop == NULL)
    {
        int err = pthread_create(&tid, NULL, &startGLoop, NULL);
        if (err != 0)
            fprintf(stderr, "Can't create thread :[%s].\n", strerror(err));
        else
            fprintf(stderr, "Thread created successfully.\n");
    }

    //TODO
    //g_main_loop_quit(event_loop);
}

bool BleClient::connect()
{
	GError *gerr = NULL;

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
        return false;
    }

    state = CONNECTING;
    fprintf(stderr, "Connecting from %s to %s with type %s\n",
            src.c_str(),
            dstAddr.c_str(),
            dstType.c_str());
    return true;
}

void BleClient::disconnect()
{
	g_attrib_unref(attrib);
	attrib = NULL;

	g_io_channel_shutdown(iochannel, FALSE, NULL);
	g_io_channel_unref(iochannel);
	iochannel = NULL;

    state = DISCONNECTED;
}

void BleClient::gattWriteCmd(const uint16_t _handle,
        const uint8_t *_data, const int _size)
{
    if(isConnected() == false)
        return;

    gatt_write_cmd(attrib, _handle, _data, _size, NULL, NULL);
}

void BleClient::gattListenToHandle(const uint16_t _handle,
        void (*cbFct)(int, const uint8_t *))
{
    std::vector<void(*)(int,const uint8_t*)> cbFcts;
    if(handleToCbFct.find(_handle) != handleToCbFct.end())
        cbFcts = handleToCbFct[_handle];
    cbFcts.push_back(cbFct);
    handleToCbFct[_handle] = cbFcts;
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
            fprintf(stderr, "Notification handle = 0x%04X value: ", handle);
            break;
        default:
            fprintf(stderr, "Invalid opcode.\n");
            return;
    }

    for(i = 3; i < len; i++)
    {
        fprintf(stderr, "%02X ", pdu[i]);
    }
    fprintf(stderr, "\n");

    if(handleToCbFct.find(handle) == handleToCbFct.end())
        return;
    std::vector<void(*)(int,const uint8_t*)> cbFcts = handleToCbFct[handle];

    for(int i = 0; i < cbFcts.size(); i++)
    {
        void (*fct)(int, const uint8_t*);
        fct = cbFcts[i];
        (*fct)(len-3, &pdu[3]);
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

    fprintf(stderr, "Characteristics value/descriptor: ");
    for(i = 0; i < vlen; i++)
    {
        fprintf(stderr, "%02X ", value[i]);
    }
    fprintf(stderr, "\n");
}

gboolean BleClient::listen_start(gpointer user_data)
{
    GAttrib *attrib = (GAttrib *)user_data;

    g_attrib_register(attrib, ATT_OP_HANDLE_NOTIFY, GATTRIB_ALL_HANDLES,
            events_handler, attrib, NULL);

    return false;
}
