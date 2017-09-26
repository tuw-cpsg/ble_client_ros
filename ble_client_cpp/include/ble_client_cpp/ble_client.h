#ifndef BLE_CLIENT_CPP_H
#define BLE_CLIENT_CPP_H

#include <glib.h>
#include <string>
#include <map>
#include <vector>

extern "C"
{
#include <gatt/bluetooth.h>
#include <gatt/btio.h>
#include <gatt/lib/uuid.h>
#include <gatt/att.h>
#include <gatt/lib/sdp.h>
#include <gatt/gattrib.h>
#include <gatt/gatt.h>
#include <gatt/src/shared/util.h>
}

typedef enum
{
    DISCONNECTED,
    CONNECTED,
    CONNECTING
} state_t;

class BleClient
{
    public:
        BleClient(const std::string &_src,
                const std::string &_dstAddr,
                const std::string &_dstType);

        bool connect();
        void disconnect();
        void gattWriteCmd(const uint16_t _handle,
                const uint8_t *_data,
                const int _size);
        void gattListenToHandle(const uint16_t _handle,
                void (*)(int,const uint8_t*));

        bool isConnected();

    private:
        std::string src;
        std::string dstAddr;
        std::string dstType;

        static GMainLoop *event_loop;
        pthread_t tid;

        GIOChannel *iochannel;
        GAttrib *attrib;
        state_t state;

        void finishConnection(void);

        static std::map<GIOChannel *, BleClient *> ioMap;
        static std::map<uint16_t, std::vector<void(*)(int,const uint8_t*)>> handleToCbFct;

        static void * startGLoop(void *);

        static void connect_cb(GIOChannel *io, GError *err, gpointer user_data);
        static gboolean listen_start(gpointer user_data);
        static void events_handler(const uint8_t *pdu, uint16_t len, gpointer user_data);
        static void char_read_cb(guint8 status, const guint8 *pdu, guint16 plen,
                gpointer user_data);
};

#endif // BLE_CLIENT_CPP_H
