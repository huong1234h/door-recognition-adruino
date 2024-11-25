#include "esp_camera.h"
#include <WiFi.h>
#define CAMERA_MODEL_AI_THINKER
#include <WebServer.h>
#include "fd_forward.h" 
#include "fr_forward.h"
#include "fr_flash.h"

const char* ssid = "LAPTOP-8L3DTC9N 3182"; // enter the Nam of wifi
const char* password = "{3N9488b"; // enter the password
void startCameraServer();
#define relayPin 2 // pin 12 can also be used
unsigned long currentMillis = 0;
unsigned long openedMillis = 0;
long interval = 10000;           // open lock for ... milliseconds
bool doorOpen = false;
#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7
WebServer server(80); 

int ls1 = 12; // limited switch 1
int ls2 = 13; // limited switch 2
int T = 2000;
int in1 = 14;
int in2 = 15;
int rls1;
int rls2;
int x = 0;
void goback() {
  digitalWrite(in1, 0);
  digitalWrite(in2, 1);
 
}

void handleCapture() {
  camera_fb_t * fb = esp_camera_fb_get(); // Capture image from the camera
  if (!fb) {
    Serial.println("Camera capture failed");
    server.send(500, "text/plain", "Failed to capture image");
    return;
  }

  // Respond with the image
  server.sendHeader("Content-Type", "image/jpeg");
  server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);

  // Return frame buffer
  esp_camera_fb_return(fb);
}
void handleOpenDoor() {
  forward(); // Gọi hàm mở cửa
  delay(1000); // Mở trong 1 giây
  stop(); // Dừng lại
  server.send(200, "text/plain", "Door opened");
}
void handleCloseDoor() {
  goback(); // Gọi hàm đóng cửa
  delay(1000); // Đóng trong 1 giây
  stop(); // Dừng lại
  server.send(200, "text/plain", "Door closed");
}
void forward() {
  digitalWrite(in1, 1);
  digitalWrite(in2, 0);

  Serial.println("hhhhh connected");
}
void stop() {
  digitalWrite(in1, 0);
  digitalWrite(in2, 0);

}

static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};
  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}
mtmn_config_t mtmn_config = app_mtmn_config();
 
static face_id_list id_list = {0};
dl_matrix3du_t *image_matrix =  NULL;
camera_fb_t * fb = NULL;

dl_matrix3du_t *aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  pinMode(relayPin, OUTPUT); // set up signal pin
  digitalWrite(relayPin, LOW); 
  pinMode(ls1, INPUT);
  pinMode(ls2, INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  server.on("/open", HTTP_POST, handleOpenDoor);
  
  server.on("/close", HTTP_POST, handleCloseDoor);
  server.on("/capture", HTTP_GET, handleCapture);
  server.begin(); 
  Serial.println("Server started.");
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_pwdn = 32;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_SVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;
 
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
 
  // drop down frame size for higher initial frame rate
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
    // if (s->id.PID == OV3660_PID) {
    // s->set_vflip(s, 1);//flip it back
    // s->set_brightness(s, 1);//up the blightness just a bit
    // s->set_saturation(s, -2);//lower the saturation
  // }
  // s->set_framesize(s, FRAMESIZE_QVGA);

  face_id_init(&id_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
  read_face_id_from_flash(&id_list);// Read current face data from on-board flash
// #if defined(CAMERA_MODEL_M5STACK_WIDE)
//   s->set_vflip(s, 1);
//   s->set_hmirror(s, 1);
// #endif

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void rzoCheckForFace() {
  currentMillis = millis();

  if (run_face_recognition()) {
    Serial.println("Face recognised");
    digitalWrite(relayPin, HIGH); // Energise relay
    forward();
    delay(1000);
    while (rls2) {
      rls2 = digitalRead(ls2);
    }
    stop();
    delay(3000);
    openedMillis = millis();
    doorOpen = true;
  }

  if (doorOpen && (currentMillis - openedMillis > interval)) {
    goback();
    delay(1000);
    while (rls1) {
      rls1 = digitalRead(ls1);
    }
    stop();
    digitalWrite(relayPin, LOW); // De-energise relay
    doorOpen = false;
  }
}
 
bool run_face_recognition() {
  bool faceRecognised = false; // default
  int64_t start_time = esp_timer_get_time();
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }
 
  int64_t fb_get_time = esp_timer_get_time();
  //Serial.printf("Get one frame in %u ms.\n", (fb_get_time - start_time) / 1000); // this line can be commented out
 
  image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  uint32_t res = fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);
  if (!res) {
    Serial.println("to rgb888 failed");
    dl_matrix3du_free(image_matrix);
  }
 
  esp_camera_fb_return(fb);
 
  box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config);
 
  if (net_boxes) {
    if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK) {
 
      int matched_id = recognize_face(&id_list, aligned_face);
      if (matched_id >= 0) {
        Serial.printf("Match Face ID: %u\n", matched_id);
        faceRecognised = true; // function will now return true
      } else {
        Serial.println("No Match Found");
        matched_id = -1;
        
      }
    } else {
      Serial.println("Face Not Aligned");
    }
 
    free(net_boxes->box);
    free(net_boxes->landmark);
    free(net_boxes);
  }
 
  dl_matrix3du_free(image_matrix);
  return faceRecognised;
}

void read_value()
{
   rls1 = digitalRead(ls1);
   rls2 = digitalRead(ls2);
}

void loop() {
  read_value();
  server.handleClient();
  rzoCheckForFace();
}

