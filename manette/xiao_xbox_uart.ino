/*
   XIAO ESP32-S3 + Manette Xbox Series X/S (USB Host)
   -> UART (Serial1) -> Arduino Micro -> nRF24L01

   Envoie un paquet binaire de 19 octets a chaque rapport de la manette :
   joysticks normalises entre -1.0 et 1.0, bouton RB uniquement.

   Cablage UART :
     XIAO D6 (GPIO43, TX) -> Arduino Micro pin 0 (RX)
     XIAO GND             -> Arduino Micro GND

   Serial1 sert a la liaison avec le Micro : les messages de debug
   sont donc desactives par defaut (DEBUG_LOG = 0) pour garder le fil propre.
*/

#include <Arduino.h>

#include "soc/soc_caps.h"

#if !SOC_USB_OTG_SUPPORTED
#error "Ce programme necessite un ESP32-S3 / S2 / P4 avec USB OTG."
#endif

#include "usb/usb_host.h"

// ----------------------------------------------------
// Reglages
// ----------------------------------------------------

#define USB_XFER_SIZE 64

#define LINK_BAUD 115200
#define DEADZONE  0.10f

// 0 = fil UART propre (que des paquets)
// 1 = messages de debug sur Serial1 (meme fil que les paquets)
#define DEBUG_LOG 0

// 0 = desactive, 1 = affiche les paquets USB bruts (necessite DEBUG_LOG 1)
#define DEBUG_RAW_REPORT 0

#if DEBUG_LOG
  #define LOGLN(...) Serial1.println(__VA_ARGS__)
  #define LOGF(...)  Serial1.printf(__VA_ARGS__)
#else
  #define LOGLN(...) ((void)0)
  #define LOGF(...)  ((void)0)
#endif

// ----------------------------------------------------
// Paquet envoye a l'Arduino Micro
// ----------------------------------------------------

struct __attribute__((packed)) Packet
{
  uint8_t header;          // 0xAA
  float lx, ly, rx, ry;    // -1.0 .. 1.0
  uint8_t rb;              // 0 / 1
  uint8_t checksum;        // XOR des octets 1..17
};

static volatile bool newPacket = false;

// ----------------------------------------------------
// USB Host
// ----------------------------------------------------

static usb_host_client_handle_t usbClient = nullptr;

// ----------------------------------------------------
// Etat manette
// ----------------------------------------------------

struct XboxController
{
  usb_device_handle_t device = nullptr;

  uint8_t interfaceNumber = 0;
  uint8_t epIn = 0;
  uint8_t epOut = 0;

  uint16_t vid = 0;
  uint16_t pid = 0;

  uint16_t inMps = 0;
  uint16_t outMps = 0;

  usb_transfer_t *inTransfer = nullptr;
  usb_transfer_t *outTransfer = nullptr;

  bool connected = false;
  bool initialized = false;

  uint8_t initStep = 0;

  bool a = false;
  bool b = false;
  bool x = false;
  bool y = false;

  bool lb = false;
  bool rb = false;

  bool menu = false;
  bool view = false;

  bool ls = false;
  bool rs = false;

  bool dpadUp = false;
  bool dpadDown = false;
  bool dpadLeft = false;
  bool dpadRight = false;

  bool share = false;

  int16_t lx = 0;
  int16_t ly = 0;
  int16_t rx = 0;
  int16_t ry = 0;

  uint16_t lt = 0;
  uint16_t rt = 0;

  uint32_t packetCount = 0;
};

static XboxController *xbox = nullptr;

// ----------------------------------------------------
// Paquets GIP Xbox One / Series
// ----------------------------------------------------

static const uint8_t gipPowerOn[] =
{
  0x05, 0x20, 0x00, 0x01, 0x00
};

static const uint8_t gipSeriesInit[] =
{
  0x05, 0x20, 0x00, 0x0F,
  0x06, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x55,
  0x53, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00
};

static const uint8_t gipExtraInit[] =
{
  0x4D, 0x10, 0x00, 0x02,
  0x07, 0x00
};

// ----------------------------------------------------
// Prototypes
// ----------------------------------------------------

static void startInputTransfer();
static void sendInitPacket(uint8_t step);
static void parseXboxReport(const uint8_t *data, size_t len);
static void removeXboxController(usb_device_handle_t goneDevice);

// ----------------------------------------------------
// Envoi vers l'Arduino Micro
// ----------------------------------------------------

static float norm(int16_t v)
{
  float f = v / 32767.0f;

  if (f > 1.0f)  f = 1.0f;
  if (f < -1.0f) f = -1.0f;

  if (fabsf(f) < DEADZONE) f = 0.0f;

  return f;
}

static void sendToMicro()
{
  if (xbox == nullptr)
    return;

  Packet p;

  p.header = 0xAA;
  p.lx = norm(xbox->lx);
  p.ly = norm(xbox->ly);
  p.rx = norm(xbox->rx);
  p.ry = norm(xbox->ry);
  p.rb = xbox->rb ? 1 : 0;

  const uint8_t *b = (const uint8_t *)&p;

  uint8_t cs = 0;
  for (size_t i = 1; i < sizeof(Packet) - 1; i++)
    cs ^= b[i];

  p.checksum = cs;
  static bool led = false;
  led = !led;
  digitalWrite(LED_BUILTIN, led ? LOW : HIGH);
  Serial1.write(b, sizeof(Packet));
}

// ----------------------------------------------------
// Filtre enumeration
// ----------------------------------------------------

#if CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK

static bool enumFilter(
  const usb_device_desc_t *dev_desc,
  uint8_t *bConfigurationValue)
{
  (void)dev_desc;

  *bConfigurationValue = 1;

  return true;
}

#endif

// ----------------------------------------------------
// Recherche de l'interface Xbox
// ----------------------------------------------------

static bool findXboxInterface(
  const usb_config_desc_t *configDesc,
  uint8_t &interfaceNumber,
  uint8_t &epIn,
  uint8_t &epOut,
  uint16_t &inMps,
  uint16_t &outMps)
{
  for (uint8_t i = 0; i < configDesc->bNumInterfaces; i++)
  {
    int offset = 0;

    const usb_intf_desc_t *intf =
      usb_parse_interface_descriptor(
        configDesc,
        i,
        0,
        &offset
      );

    if (intf == nullptr)
      continue;

    // Xbox One / Series : SubClass 0x47, Protocol 0xD0
    if (intf->bInterfaceSubClass != 0x47)
      continue;

    if (intf->bInterfaceProtocol != 0xD0)
      continue;

    if (intf->bNumEndpoints < 2)
      continue;

    uint8_t foundIn = 0;
    uint8_t foundOut = 0;

    uint16_t foundInMps = 0;
    uint16_t foundOutMps = 0;

    for (uint8_t e = 0; e < intf->bNumEndpoints; e++)
    {
      const usb_ep_desc_t *ep =
        usb_parse_endpoint_descriptor_by_index(
          intf,
          e,
          configDesc->wTotalLength,
          &offset
        );

      if (ep == nullptr)
        continue;

      uint8_t transferType =
        ep->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK;

      if (transferType != USB_BM_ATTRIBUTES_XFER_INT)
        continue;

      uint16_t mps =
        ep->wMaxPacketSize & 0x07FF;

      if (ep->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK)
      {
        foundIn = ep->bEndpointAddress;
        foundInMps = mps;
      }
      else
      {
        foundOut = ep->bEndpointAddress;
        foundOutMps = mps;
      }
    }

    if (foundIn && foundOut)
    {
      interfaceNumber = intf->bInterfaceNumber;

      epIn = foundIn;
      epOut = foundOut;

      inMps = foundInMps;
      outMps = foundOutMps;

      return true;
    }
  }

  return false;
}

// ----------------------------------------------------
// Callback transfert OUT
// ----------------------------------------------------

static void outTransferCallback(usb_transfer_t *transfer)
{
  if (xbox == nullptr)
    return;

  if (transfer != xbox->outTransfer)
    return;

  if (transfer->status != USB_TRANSFER_STATUS_COMPLETED)
  {
    LOGLN("[XBOX] Erreur transfert OUT");
    return;
  }

  if (!xbox->connected)
    return;

  if (xbox->initStep == 0)
  {
    xbox->initStep = 1;

    LOGLN("[XBOX] Envoi initialisation Series X/S");

    sendInitPacket(1);
  }
  else if (xbox->initStep == 1)
  {
    if (xbox->vid == 0x045E && xbox->pid == 0x0B00)
    {
      xbox->initStep = 2;

      LOGLN("[XBOX] Envoi paquet d'initialisation supplementaire");

      sendInitPacket(2);
    }
    else
    {
      xbox->initialized = true;

      LOGLN("[XBOX] Initialisation terminee");

      startInputTransfer();
    }
  }
  else if (xbox->initStep == 2)
  {
    xbox->initialized = true;

    LOGLN("[XBOX] Initialisation terminee");

    startInputTransfer();
  }
}

// ----------------------------------------------------
// Envoie un paquet d'initialisation
// ----------------------------------------------------

static void sendInitPacket(uint8_t step)
{
  if (xbox == nullptr || !xbox->connected)
    return;

  const uint8_t *packet = nullptr;
  size_t len = 0;

  switch (step)
  {
    case 0:
      packet = gipPowerOn;
      len = sizeof(gipPowerOn);
      break;

    case 1:
      packet = gipSeriesInit;
      len = sizeof(gipSeriesInit);
      break;

    case 2:
      packet = gipExtraInit;
      len = sizeof(gipExtraInit);
      break;

    default:
      return;
  }

  if (len > xbox->outTransfer->data_buffer_size)
  {
    LOGLN("[XBOX] Paquet OUT trop grand");
    return;
  }

  memcpy(
    xbox->outTransfer->data_buffer,
    packet,
    len
  );

  xbox->outTransfer->device_handle = xbox->device;
  xbox->outTransfer->bEndpointAddress = xbox->epOut;
  xbox->outTransfer->num_bytes = len;
  xbox->outTransfer->callback = outTransferCallback;
  xbox->outTransfer->context = nullptr;

  esp_err_t err =
    usb_host_transfer_submit(xbox->outTransfer);

  if (err != ESP_OK)
  {
    LOGF("[XBOX] Erreur envoi OUT : %s\n", esp_err_to_name(err));
  }
}

// ----------------------------------------------------
// Callback transfert IN
// ----------------------------------------------------

static void inTransferCallback(usb_transfer_t *transfer)
{
  if (xbox == nullptr)
    return;

  if (transfer != xbox->inTransfer)
    return;

  if (transfer->status == USB_TRANSFER_STATUS_COMPLETED)
  {
    if (transfer->actual_num_bytes > 0)
    {
      parseXboxReport(
        transfer->data_buffer,
        transfer->actual_num_bytes
      );
    }

    if (xbox != nullptr && xbox->connected)
    {
      transfer->device_handle = xbox->device;
      transfer->bEndpointAddress = xbox->epIn;
      transfer->num_bytes = xbox->inMps;

      esp_err_t err =
        usb_host_transfer_submit(transfer);

      if (err != ESP_OK)
      {
        LOGF("[XBOX] Erreur resoumission IN : %s\n", esp_err_to_name(err));
      }
    }
  }
}

// ----------------------------------------------------
// Demarrage lecture IN
// ----------------------------------------------------

static void startInputTransfer()
{
  if (xbox == nullptr)
    return;

  xbox->inTransfer->device_handle = xbox->device;
  xbox->inTransfer->bEndpointAddress = xbox->epIn;

  xbox->inTransfer->num_bytes =
    min(
      (size_t)xbox->inMps,
      (size_t)USB_XFER_SIZE
    );

  xbox->inTransfer->callback = inTransferCallback;
  xbox->inTransfer->context = nullptr;

  esp_err_t err =
    usb_host_transfer_submit(xbox->inTransfer);

  if (err != ESP_OK)
  {
    LOGF("[XBOX] Impossible de demarrer IN : %s\n", esp_err_to_name(err));
  }
}

// ----------------------------------------------------
// Affichage paquet brut
// ----------------------------------------------------

static void dumpRawReport(
  const uint8_t *data,
  size_t len)
{
#if DEBUG_LOG && DEBUG_RAW_REPORT

  Serial1.print("[RAW] ");

  for (size_t i = 0; i < len; i++)
  {
    if (data[i] < 0x10)
      Serial1.print('0');

    Serial1.print(data[i], HEX);
    Serial1.print(' ');
  }

  Serial1.println();

#else

  (void)data;
  (void)len;

#endif
}

// ----------------------------------------------------
// Lecture d'un rapport Xbox Series X/S
// ----------------------------------------------------

static void parseXboxReport(
  const uint8_t *data,
  size_t len)
{
  if (xbox == nullptr)
    return;

  if (len < 18)
    return;

  if (data[0] != 0x20)
    return;

  xbox->packetCount++;

  dumpRawReport(data, len);

  // Boutons
  uint16_t buttons =
    (uint16_t)data[4] |
    ((uint16_t)data[5] << 8);

  xbox->dpadUp    = buttons & (1 << 8);
  xbox->dpadDown  = buttons & (1 << 9);
  xbox->dpadLeft  = buttons & (1 << 10);
  xbox->dpadRight = buttons & (1 << 11);

  xbox->menu = buttons & (1 << 2);
  xbox->view = buttons & (1 << 3);

  xbox->ls = buttons & (1 << 14);
  xbox->rs = buttons & (1 << 15);

  xbox->lb = buttons & (1 << 12);
  xbox->rb = buttons & (1 << 13);

  xbox->a = buttons & (1 << 4);
  xbox->b = buttons & (1 << 5);
  xbox->x = buttons & (1 << 6);
  xbox->y = buttons & (1 << 7);

  // Gachettes
  if (len >= 10)
  {
    uint16_t rawLT =
      (uint16_t)data[6] |
      ((uint16_t)data[7] << 8);

    uint16_t rawRT =
      (uint16_t)data[8] |
      ((uint16_t)data[9] << 8);

    xbox->lt = rawLT >> 2;
    xbox->rt = rawRT >> 2;

    if (xbox->lt > 1023)
      xbox->lt = 1023;

    if (xbox->rt > 1023)
      xbox->rt = 1023;
  }

  // Joystick gauche
  xbox->lx =
    (int16_t)(
      (uint16_t)data[10] |
      ((uint16_t)data[11] << 8)
    );

  xbox->ly =
    (int16_t)(
      (uint16_t)data[12] |
      ((uint16_t)data[13] << 8)
    );

  // Joystick droit
  xbox->rx =
    (int16_t)(
      (uint16_t)data[14] |
      ((uint16_t)data[15] << 8)
    );

  xbox->ry =
    (int16_t)(
      (uint16_t)data[16] |
      ((uint16_t)data[17] << 8)
    );

  // Bouton Share
  xbox->share = false;

  if (len == 48)
  {
    xbox->share = (data[22] & 0x01);
  }
  else if (len == 36 || len == 44)
  {
    xbox->share = (data[18] & 0x01);
  }

  // Un nouveau paquet est pret a etre envoye a l'Arduino Micro
  newPacket = true;
}

// ----------------------------------------------------
// Ajout d'une manette
// ----------------------------------------------------

static void deviceAdded(uint8_t address)
{
  if (xbox != nullptr)
  {
    LOGLN("[USB] Une manette est deja ouverte");
    return;
  }

  usb_device_handle_t device = nullptr;

  esp_err_t err =
    usb_host_device_open(
      usbClient,
      address,
      &device
    );

  if (err != ESP_OK)
  {
    LOGF("[USB] Ouverture impossible : %s\n", esp_err_to_name(err));

    return;
  }

  const usb_device_desc_t *deviceDesc = nullptr;

  err =
    usb_host_get_device_descriptor(
      device,
      &deviceDesc
    );

  if (err != ESP_OK)
  {
    LOGLN("[USB] Device descriptor impossible");

    usb_host_device_close(usbClient, device);

    return;
  }

  LOGLN();
  LOGLN("==============================");
  LOGLN("Nouveau peripherique USB");
  LOGF("VID = 0x%04X\n", deviceDesc->idVendor);
  LOGF("PID = 0x%04X\n", deviceDesc->idProduct);
  LOGLN("==============================");

  const usb_config_desc_t *configDesc = nullptr;

  err =
    usb_host_get_active_config_descriptor(
      device,
      &configDesc
    );

  if (err != ESP_OK)
  {
    LOGLN("[USB] Configuration descriptor impossible");

    usb_host_device_close(usbClient, device);

    return;
  }

  uint8_t interfaceNumber = 0;
  uint8_t epIn = 0;
  uint8_t epOut = 0;

  uint16_t inMps = 0;
  uint16_t outMps = 0;

  if (!findXboxInterface(
        configDesc,
        interfaceNumber,
        epIn,
        epOut,
        inMps,
        outMps))
  {
    LOGLN("[USB] Ce peripherique n'est pas une interface Xbox GIP");

    usb_host_device_close(usbClient, device);

    return;
  }

  LOGLN("[XBOX] Interface Xbox GIP detectee");
  LOGF("[XBOX] Interface : %u\n", interfaceNumber);
  LOGF("[XBOX] Endpoint IN  : 0x%02X\n", epIn);
  LOGF("[XBOX] Endpoint OUT : 0x%02X\n", epOut);
  LOGF("[XBOX] IN MPS  : %u\n", inMps);
  LOGF("[XBOX] OUT MPS : %u\n", outMps);

  XboxController *c = new XboxController();

  if (c == nullptr)
  {
    LOGLN("[XBOX] Plus assez de RAM");

    usb_host_device_close(usbClient, device);

    return;
  }

  c->device = device;
  c->interfaceNumber = interfaceNumber;

  c->epIn = epIn;
  c->epOut = epOut;

  c->inMps = inMps;
  c->outMps = outMps;

  c->vid = deviceDesc->idVendor;
  c->pid = deviceDesc->idProduct;

  c->connected = true;

  // Claim interface
  err =
    usb_host_interface_claim(
      usbClient,
      device,
      interfaceNumber,
      0
    );

  if (err != ESP_OK)
  {
    LOGF("[XBOX] Impossible de prendre l'interface : %s\n", esp_err_to_name(err));

    delete c;

    usb_host_device_close(usbClient, device);

    return;
  }

  // Transfer IN
  err =
    usb_host_transfer_alloc(
      USB_XFER_SIZE,
      0,
      &c->inTransfer
    );

  if (err != ESP_OK)
  {
    LOGLN("[XBOX] Allocation transfert IN impossible");

    usb_host_interface_release(usbClient, device, interfaceNumber);

    delete c;

    usb_host_device_close(usbClient, device);

    return;
  }

  // Transfer OUT
  err =
    usb_host_transfer_alloc(
      USB_XFER_SIZE,
      0,
      &c->outTransfer
    );

  if (err != ESP_OK)
  {
    LOGLN("[XBOX] Allocation transfert OUT impossible");

    usb_host_transfer_free(c->inTransfer);

    usb_host_interface_release(usbClient, device, interfaceNumber);

    delete c;

    usb_host_device_close(usbClient, device);

    return;
  }

  xbox = c;

  LOGLN("[XBOX] Manette connectee");

  xbox->initStep = 0;

  sendInitPacket(0);
}

// ----------------------------------------------------
// Retrait manette
// ----------------------------------------------------

static void removeXboxController(
  usb_device_handle_t goneDevice)
{
  if (xbox == nullptr)
    return;

  if (xbox->device != goneDevice)
    return;

  XboxController *c = xbox;

  // Les callbacks tardifs seront ignores
  xbox = nullptr;

  c->connected = false;

  LOGLN();
  LOGLN("[XBOX] Manette deconnectee");

  if (c->epIn)
  {
    usb_host_endpoint_halt(c->device, c->epIn);
    usb_host_endpoint_flush(c->device, c->epIn);
  }

  if (c->epOut)
  {
    usb_host_endpoint_halt(c->device, c->epOut);
    usb_host_endpoint_flush(c->device, c->epOut);
  }

  usb_host_interface_release(
    usbClient,
    c->device,
    c->interfaceNumber
  );

  usb_host_device_close(
    usbClient,
    c->device
  );

  if (c->inTransfer)
    usb_host_transfer_free(c->inTransfer);

  if (c->outTransfer)
    usb_host_transfer_free(c->outTransfer);

  delete c;
}

// ----------------------------------------------------
// Evenements USB Host
// ----------------------------------------------------

static void clientEvent(
  const usb_host_client_event_msg_t *msg,
  void *arg)
{
  (void)arg;

  switch (msg->event)
  {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
      deviceAdded(msg->new_dev.address);
      break;

    case USB_HOST_CLIENT_EVENT_DEV_GONE:
      removeXboxController(msg->dev_gone.dev_hdl);
      break;

    default:
      break;
  }
}

// ----------------------------------------------------
// Tache daemon USB Host
// ----------------------------------------------------

static void usbDaemonTask(void *arg)
{
  (void)arg;

  while (true)
  {
    uint32_t flags = 0;

    usb_host_lib_handle_events(
      portMAX_DELAY,
      &flags
    );

    if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
    {
      usb_host_device_free_all();
    }
  }
}

// ----------------------------------------------------
// SETUP
// ----------------------------------------------------

void setup()
{
  // Serial1 remappe sur GPIO44 (RX, D7) / GPIO43 (TX, D6)
  // Liaison UART vers l'Arduino Micro
  Serial1.begin(LINK_BAUD, SERIAL_8N1, 44, 43);   // RX=GPIO44 (D7), TX=GPIO43 (D6)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);   // LED éteinte (active à l'état bas)
  delay(1000);

  LOGLN();
  LOGLN("==============================");
  LOGLN("XIAO ESP32-S3 Xbox USB Host");
  LOGLN("==============================");

  usb_host_config_t hostConfig = {};

  hostConfig.intr_flags = ESP_INTR_FLAG_LEVEL1;

#if CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK
  hostConfig.enum_filter_cb = enumFilter;
#endif

  esp_err_t err = usb_host_install(&hostConfig);

  if (err != ESP_OK)
  {
    LOGF("usb_host_install() : %s\n", esp_err_to_name(err));

    return;
  }

  usb_host_client_config_t clientConfig = {};

  clientConfig.is_synchronous = false;
  clientConfig.max_num_event_msg = 8;

  clientConfig.async.client_event_callback = clientEvent;
  clientConfig.async.callback_arg = nullptr;

  err = usb_host_client_register(
    &clientConfig,
    &usbClient
  );

  if (err != ESP_OK)
  {
    LOGF("usb_host_client_register() : %s\n", esp_err_to_name(err));

    return;
  }

  xTaskCreate(
    usbDaemonTask,
    "USB_DAEMON",
    4096,
    nullptr,
    2,
    nullptr
  );

  LOGLN();
  LOGLN("USB Host pret.");
  LOGLN("Branche la manette Xbox...");
}

// ----------------------------------------------------
// LOOP
// ----------------------------------------------------

void loop()
{
  if (usbClient != nullptr)
  {
    usb_host_client_handle_events(usbClient, 0);
  }

  // Envoi UART des qu'un nouveau rapport manette est disponible
  if (newPacket && xbox != nullptr && xbox->initialized)
  {
    newPacket = false;

    sendToMicro();
  }

  delay(1);
}
