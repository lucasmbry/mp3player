#include <string>
#include <vector>
#include <atomic>
#include <mutex>

// ESP-IDF diagnostics used by the logging helpers
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_chip_info.h"

// Arduino core (setup/loop, String, millis(), initArduino, FreeRTOS hooks)
#include "Arduino.h"

// File system
#include "FS.h"

// Hardware 
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ST7789.h"
#include "Adafruit_ILI9341.h"
#include "ESP32Encoder.h"
#include "TJpg_Decoder.h"

// ----------------------------------------------------------------- Logging
#define DEBUG_LOGGING 1

#define LOG_RAW(level, fmt, ...) \
    printf("[%8lu|C%d|%c] " fmt "\n", \
           (unsigned long)(esp_timer_get_time() / 1000), (int)xPortGetCoreID(), level, ##__VA_ARGS__)

#define logi(fmt, ...) LOG_RAW('I', fmt, ##__VA_ARGS__)
#define logw(fmt, ...) LOG_RAW('W', fmt, ##__VA_ARGS__)
#define loge(fmt, ...) LOG_RAW('E', fmt, ##__VA_ARGS__)
#if DEBUG_LOGGING
#define logd(fmt, ...) LOG_RAW('D', fmt, ##__VA_ARGS__)
#else
#define logd(fmt, ...) do {} while (0)
#endif

struct ScopedTimer {
    const char* name;
    int64_t startUs;
    explicit ScopedTimer(const char* n) : name(n), startUs(esp_timer_get_time()) {
        logd("[timer] %s: start", name);
    }
    ~ScopedTimer() {
        logi("[timer] %s: took %lld ms", name, (long long)((esp_timer_get_time() - startUs) / 1000));
    }
};

static void logHeap(const char* tag) {
    logi("[mem] %s: free heap %u, free PSRAM %u",
         tag, (unsigned)esp_get_free_heap_size(), (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

// ----- ESP 32 Hardware Pins -----
#define TFT_MOSI 11
#define TFT_MISO 13
#define TFT_SCLK 12
#define TFT_DC    9
#define TFT_RST   8
#define TFT1_CS  10
#define TFT2_CS  14
#define ENC_CLK   4
#define ENC_DT    5
#define ENC_SW    6

Adafruit_ST7789 tft1 = Adafruit_ST7789(TFT1_CS, TFT_DC, TFT_RST);
Adafruit_ST7789 tft2 = Adafruit_ST7789(TFT2_CS, TFT_DC, -1);
ESP32Encoder encoder;

#define TFT1_SPI_HZ 10000000

template <typename T>
auto setTftSpeed(T& t, uint32_t hz, int) -> decltype(t.setSPISpeed(hz), void()) { t.setSPISpeed(hz); }
template <typename T>
void setTftSpeed(T&, uint32_t, long) { printf("[setup] this Adafruit lib has no setSPISpeed(), SPI speed unchanged\n"); }

// ----- MP3 / USB Player State -----
// arduino-esp32 4.x dropped the USB Host MSC library, so no real USB
// host driver is available in this build yet. This stub keeps the
// player compiling/booting (UI + encoder work, playlist stays empty).
// Replace with a real USB-host MSC driver when it is available.
class MscStub {
public:
    void begin() {
        logi("[USB] USB host MSC not available in this build (stub)");
    }
    bool isReady() const { return false; }
    File open(const char* /*path*/) { return File(); }
};
MscStub msc;
std::vector<String> playlist;
volatile int currentTrackIndex = 0;

volatile bool isPlaying = false;
volatile long simulatedProgressMs = 0;
volatile long simulatedDurationMs = 300000; // Fake duration: 5 minutes for now
std::mutex stateMutex;

volatile bool needsUiUpdate = true;
volatile bool tft2UiDirty = true;
int lastEncoderCount = 0;

// Debug counters
static uint32_t dbgLoopCount = 0;

// ----- TFT Configurations -----
#define ART_SWAP_BYTES 0
#define ART_RGB888 1   
#define ART_PAD 5
#define ART_CORNER_RADIUS 10

#define TFT2_SHOW_DEBUG_DIVISORS 0   
#define COLOR_BG      ILI9341_BLACK
#define COLOR_TEXT    ILI9341_WHITE
#define COLOR_SUBTEXT 0x9CD3          
#define COLOR_TRACK_BG 0x39E7         

static int artSize = 0;   
static int artSrcW = 0;   
static int artSrcH = 0;   

// TJpgDec Output mapping logic
bool tft1_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
    static uint16_t buf[1024];  
    if (artSize <= 0 || artSrcW <= 0 || artSrcH <= 0) return 1;

    int dx0 = ((int)x * artSize + artSrcW - 1) / artSrcW;       
    int dx1 = (((int)(x + w) * artSize) - 1) / artSrcW;         
    int dy0 = ((int)y * artSize + artSrcH - 1) / artSrcH;
    int dy1 = (((int)(y + h) * artSize) - 1) / artSrcH;
    if (dx1 >= artSize) dx1 = artSize - 1;
    if (dy1 >= artSize) dy1 = artSize - 1;

    int cols = dx1 - dx0 + 1;
    int rows = dy1 - dy0 + 1;
    if (cols <= 0 || rows <= 0) return 1;

    int rowsPerChunk = 1024 / cols;
    if (rowsPerChunk < 1) return 1;

    const int r_sq = ART_CORNER_RADIUS * ART_CORNER_RADIUS;

    for (int dy = dy0; dy <= dy1; dy += rowsPerChunk) {
        int n = dy1 - dy + 1;
        if (n > rowsPerChunk) n = rowsPerChunk;

        for (int r = 0; r < n; r++) {
            const int targetY = dy + r;
            const int sy = (targetY * artSrcH) / artSize - y;  
            uint16_t* out = buf + r * cols;

            for (int i = 0; i < cols; i++) {
                const int targetX = dx0 + i;
                bool outsideCorner = false;
                if (ART_CORNER_RADIUS > 0) {
                    int cx = -1, cy = -1;
                    if (targetX < ART_CORNER_RADIUS) cx = ART_CORNER_RADIUS - 1;
                    else if (targetX >= artSize - ART_CORNER_RADIUS) cx = artSize - ART_CORNER_RADIUS;

                    if (targetY < ART_CORNER_RADIUS) cy = ART_CORNER_RADIUS - 1;
                    else if (targetY >= artSize - ART_CORNER_RADIUS) cy = artSize - ART_CORNER_RADIUS;

                    if (cx != -1 && cy != -1) {
                        int dist_sq = (targetX - cx) * (targetX - cx) + (targetY - cy) * (targetY - cy);
                        if (dist_sq > r_sq) outsideCorner = true;
                    }
                }

                if (outsideCorner) {
                    out[i] = ILI9341_BLACK;
                } else {
                    const int sx = (targetX * artSrcW) / artSize - x;
#if ART_RGB888
                    const uint8_t* p = (const uint8_t*)bitmap + ((size_t)sy * w + sx) * 3;
                    uint16_t px = ((p[0] & 0xF8) << 8) | ((p[1] & 0xFC) << 3) | (p[2] >> 3);
#else
                    uint16_t px = bitmap[(size_t)sy * w + sx];
#endif
#if ART_SWAP_BYTES
                    px = (uint16_t)((px << 8) | (px >> 8));
#endif
                    out[i] = px;
                }
            }
        }
        tft1.startWrite();
        tft1.setAddrWindow(dx0 + ART_PAD, dy + ART_PAD, cols, n);
        tft1.writePixels(buf, cols * n);
        tft1.endWrite();
    }
    return 1;
}

// ----------------------------------------------------------------- Local File Scanner

void scanUSBForMP3s() {
    logi("[USB] Scanning for MP3 files...");
    File root = msc.open("/");
    if (!root || !root.isDirectory()) {
        loge("[USB] Failed to open root directory");
        return;
    }

    std::lock_guard<std::mutex> lock(stateMutex);
    playlist.clear();

    File file = root.openNextFile();
    while (file) {
        if (!file.isDirectory()) {
            String fileName = file.name();
            // Simple check for .mp3
            if (fileName.endsWith(".mp3") || fileName.endsWith(".MP3")) {
                playlist.push_back(fileName);
                logd("[USB] Found MP3: %s", fileName.c_str());
            }
        }
        file = root.openNextFile();
    }
    
    logi("[USB] Scan complete, found %d MP3 files", (int)playlist.size());
    if (playlist.size() > 0) {
        currentTrackIndex = 0;
        simulatedProgressMs = 0;
        needsUiUpdate = true;
    }
}

// ----------------------------------------------------------------- Audio Engine CORE 0

void audioTask(void* pvParameters) {
    logi("[Audio] Initializing Audio Engine on Core 0...");
    logHeap("audioTask start");

    // Wait for USB Drive to be mounted
    while (!msc.isReady()) {
        logi("[Audio] Waiting for USB Drive...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    logi("[Audio] USB Drive Ready! Scanning...");
    scanUSBForMP3s();

    uint32_t ticks = 0;
    while (true) {
        // --- DUMMY AUDIO SINK ---
        // Right now, this just advances the fake progress timer. 
        // Later, you'll open the MP3 file here, decode it chunk by chunk, and push it to I2S.
        
        if (isPlaying && playlist.size() > 0) {
            simulatedProgressMs += 100; // Increment progress by 100ms
            
            // Auto skip to next track when finished
            if (simulatedProgressMs >= simulatedDurationMs) {
                std::lock_guard<std::mutex> lock(stateMutex);
                currentTrackIndex = (currentTrackIndex + 1) % playlist.size();
                simulatedProgressMs = 0;
                needsUiUpdate = true;
                logi("[Audio] Track finished, skipping to next.");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Run at ~10 Hz

        if (++ticks % 300 == 0) { // every ~30s
            logd("[Audio] Task alive, stack HWM %u bytes", (unsigned)uxTaskGetStackHighWaterMark(NULL));
        }
    }
}

// ---------------------------------------------------------------------------------- UI CORE 1

// ---- Progress bar state ----
static long          progBaseMs     = 0;   
static long          progDurationMs = 1;
static unsigned long progBaseTime   = 0;   
static bool          progPlaying    = false;
static int           barLastFillW   = -1;  

// ---- Text animation state ----
#define UI_TITLE_TEXT_SIZE   2
#define UI_SUBTEXT_TEXT_SIZE 1
#define UI_TITLE_Y           4
#define UI_SUBTEXT_Y         24
#define UI_TITLE_CHAR_W      (6 * UI_TITLE_TEXT_SIZE)    
#define UI_SUBTEXT_CHAR_W    (6 * UI_SUBTEXT_TEXT_SIZE)  
#define UI_TITLE_SPEED_PX_S  24
#define UI_ROT_HOLD_MS       2500
#define UI_ROT_FADE_IN_MS    250
#define UI_ROT_FADE_OUT_MS   250

static std::string   uiTitle, uiArtist, uiAlbum;
static bool          uiTitleNeedsScroll = false;
static unsigned long uiTitleScrollBase = 0;
static int           uiTitleLastX      = 0;
static unsigned long uiRotBase         = 0;  
static int           uiRotLastSide     = -1;
static int           uiRotLastFade     = -1; 

static void drawPillColumns(Adafruit_GFX& tft, int x, int y, int w, int h,
                            int c0, int c1, int fillW, uint16_t fg, uint16_t bg) {
    float r = h / 2.0f;
    if (r * 2 > w) r = w / 2.0f;
    if (c0 < 0) c0 = 0;
    if (c1 > w) c1 = w;

    for (int i = c0; i < c1; i++) {
        float px = i + 0.5f;
        float dx = 0;
        if (px < r)          dx = r - px;
        else if (px > w - r) dx = px - (w - r);

        float half = (dx > 0) ? sqrtf(r * r - dx * dx) : r;
        int yStart = (int)(h / 2.0f - half + 0.5f);
        int yEnd   = (int)(h / 2.0f + half + 0.5f);
        int len = yEnd - yStart;
        if (len < 1) len = 1;

        tft.drawFastVLine(x + i, y + yStart, len, (i < fillW) ? fg : bg);
    }
}

static void updateProgressBar(bool force = false) {
    const int W = tft2.width();
    const int halfH = tft2.height() / 2;
    const int barMarginX = 6;
    const int barHeight  = 16;
    const int barWidth   = W - (barMarginX * 2);
    const int barX       = barMarginX;
    const int barY       = halfH + (halfH - barHeight) / 2;

    long pos = progBaseMs;
    if (progPlaying) pos += (long)(millis() - progBaseTime);
    if (pos < 0) pos = 0;
    if (pos > progDurationMs) pos = progDurationMs;

    int fillW = (int)(((int64_t)pos * barWidth) / progDurationMs);

    if (force || barLastFillW < 0) {
        drawPillColumns(tft2, barX, barY, barWidth, barHeight, 0, barWidth,
                        fillW, COLOR_TEXT, COLOR_TRACK_BG);
    } else if (fillW != barLastFillW) {
        int lo = (fillW < barLastFillW) ? fillW : barLastFillW;
        int hi = (fillW < barLastFillW) ? barLastFillW : fillW;
        drawPillColumns(tft2, barX, barY, barWidth, barHeight, lo, hi,
                        fillW, COLOR_TEXT, COLOR_TRACK_BG);
    }
    barLastFillW = fillW;
}

static int uiLeftSectionW() {
    const int W = tft2.width();
    return W - (W * 20) / 100;
}

static uint16_t uiBlend565(uint16_t a, uint16_t b, int t) {
    int ar = (a >> 11) & 31, ag = (a >> 5) & 63, ab = a & 31;
    int br = (b >> 11) & 31, bg = (b >> 5) & 63, bb = b & 31;
    int rr = ar + ((br - ar) * t) / 255;
    int gg = ag + ((bg - ag) * t) / 255;
    int bl = ab + ((bb - ab) * t) / 255;
    if (rr < 0) rr = 0; if (rr > 31) rr = 31;
    if (gg < 0) gg = 0; if (gg > 63) gg = 63;
    if (bl < 0) bl = 0; if (bl > 31) bl = 31;
    return (uint16_t)((rr << 11) | (gg << 5) | bl);
}

static void drawTitleFrame() {
    if (!uiTitleNeedsScroll) return;

    const int regionLeft  = 4;
    const int regionRight = uiLeftSectionW();
    const int regionW     = regionRight - regionLeft;
    const int textW       = (int)uiTitle.size() * UI_TITLE_CHAR_W;
    const int gap         = 30;
    const int travel      = regionW + textW + gap;

    unsigned long elapsed = millis() - uiTitleScrollBase;
    int o = (int)(((unsigned long long)elapsed * UI_TITLE_SPEED_PX_S / 1000ULL) % (unsigned long long)travel);
    int x = regionRight - o;   

    if (x == uiTitleLastX) return;   
    uiTitleLastX = x;

    tft2.fillRect(regionLeft, UI_TITLE_Y, regionW, 16, COLOR_BG);
    tft2.setTextColor(COLOR_TEXT);
    tft2.setTextSize(UI_TITLE_TEXT_SIZE);
    const int len = (int)uiTitle.size();
    for (int i = 0; i < len; i++) {
        int charX = x + i * UI_TITLE_CHAR_W;
        if (charX < regionLeft || charX + UI_TITLE_CHAR_W > regionRight) continue;
        tft2.setCursor(charX, UI_TITLE_Y);
        tft2.print(uiTitle[i]);
    }
}

static void drawSubTextFrame() {
    const int regionLeft = 4;
    const int regionW    = uiLeftSectionW() - regionLeft;

    const unsigned long HOLD    = UI_ROT_HOLD_MS;
    const unsigned long FADEIN  = UI_ROT_FADE_IN_MS;
    const unsigned long FADEOUT = UI_ROT_FADE_OUT_MS;
    const unsigned long L0      = HOLD + FADEOUT;  
    const unsigned long L1      = FADEIN + HOLD;   
    const unsigned long cycle   = L0 + L1;

    unsigned long t  = millis() - uiRotBase;
    unsigned long tt = t % cycle;
    int side; unsigned long tp;
    if (tt < L0) { side = 0; tp = tt; }
    else         { side = 1; tp = tt - L0; }

    int fade;
    if (side == 0) fade = (tp < HOLD) ? 255 : (255 - (int)((tp - HOLD) * 255 / FADEOUT));
    else           fade = (tp < FADEIN) ? (int)(tp * 255 / FADEIN) : 255;
    if (fade < 0) fade = 0; if (fade > 255) fade = 255;

    if (side == uiRotLastSide && fade == uiRotLastFade) return;
    uiRotLastSide = side;
    uiRotLastFade = fade;

    const std::string& text = (side == 0) ? uiArtist : uiAlbum;
    tft2.fillRect(regionLeft, UI_SUBTEXT_Y, regionW, 8, COLOR_BG);
    tft2.setTextColor(uiBlend565(COLOR_BG, COLOR_SUBTEXT, fade));
    tft2.setTextSize(UI_SUBTEXT_TEXT_SIZE);
    int maxChars = regionW / UI_SUBTEXT_CHAR_W;
    if (maxChars < 0) maxChars = 0;
    tft2.setCursor(regionLeft, UI_SUBTEXT_Y);
    tft2.print(String(text.substr(0, maxChars).c_str()));
}

static void updateTextAnimations() {
    drawTitleFrame();
    drawSubTextFrame();
}

std::string utf8ToAscii(std::string s) {
    std::string out;
    out.reserve(s.size());
    for (size_t i = 0; i < s.size(); i++) {
        if ((uint8_t)s[i] < 0x80) out += s[i];
        // Note: Simple stub. Actual multibyte sequence handling can be added here if needed.
    }
    return out;
}

void updateUI(std::string title, std::string artist, std::string album, int trackNum, int totalTracks, long progress, long duration, bool playing = false) {
    title = utf8ToAscii(std::move(title));
    artist = utf8ToAscii(std::move(artist));
    album = utf8ToAscii(std::move(album));

    const int W = tft2.width();
    const int H = tft2.height();
    const int halfH = H / 2;
    const int rightSectionW = (W * 20) / 100;
    const int leftSectionW  = W - rightSectionW;

    static int lastTrackNum = -1, lastTotalTracks = -1;

    progBaseMs     = progress;
    progDurationMs = (duration > 0) ? duration : 1;
    progBaseTime   = millis();
    progPlaying    = playing;

    bool metaChanged = tft2UiDirty || title != uiTitle || artist != uiArtist || album != uiAlbum ||
                       trackNum != lastTrackNum || totalTracks != lastTotalTracks;

    if (metaChanged) {
        uiTitle = title;
        uiArtist = artist;
        uiAlbum = album;
        uiTitleNeedsScroll = ((int)title.size() * UI_TITLE_CHAR_W) > (leftSectionW - 8);
        uiTitleScrollBase = millis();
        uiTitleLastX = 0;
        uiRotBase = millis();
        uiRotLastSide = -1;
        uiRotLastFade = -1;

        tft2.fillRect(0, 0, W, halfH, COLOR_BG);

        if (uiTitleNeedsScroll) {
            drawTitleFrame();
        } else {
            tft2.setTextColor(COLOR_TEXT);
            tft2.setTextSize(UI_TITLE_TEXT_SIZE);
            tft2.setCursor(4, UI_TITLE_Y);
            tft2.print(uiTitle.c_str());
        }

        drawSubTextFrame();

        if (totalTracks > 0) {
            char posBuf[16];
            snprintf(posBuf, sizeof(posBuf), "%d/%d", trackNum, totalTracks);
            int charCount = strlen(posBuf);
            int textSize = (charCount * 12 <= rightSectionW - 4) ? 2 : 1;
            int textW = charCount * (textSize == 2 ? 12 : 6);
            int textH = (textSize == 2 ? 16 : 8);
            int textX = leftSectionW + (rightSectionW - textW) / 2;
            int textY = (halfH - textH) / 2;

            tft2.setTextColor(COLOR_TEXT);
            tft2.setTextSize(textSize);
            tft2.setCursor(textX, textY);
            tft2.print(posBuf);
        }

        tft2.fillRect(0, halfH, W, halfH, COLOR_BG);

        lastTrackNum = trackNum;
        lastTotalTracks = totalTracks;
        tft2UiDirty = false;
    }

    updateProgressBar(metaChanged);
}

// Draw album art locally if ID3 tag bytes are parsed later
void drawAlbumArtFromBuffer(const uint8_t* buffer, size_t len) {
    if(!buffer || len == 0) {
        // Clear screen if no art
        tft1.fillScreen(ILI9341_BLACK);
        return;
    }
    
    // (You can port the JPEG decode logic here to read from the memory buffer directly instead of HTTP client)
    JRESULT jr = TJpgDec.drawJpg(0, 0, buffer, len);
    if (jr != JDR_OK) loge("[Art] Local JPEG decode failed: %d", (int)jr);
}

// ---------------------------------------------------------------------------------- Lifecycle

void setup() {
    Serial.begin(115200);
    logi("=====================================================");
    logi("[setup] Booting MP3 Box");
    logHeap("boot");

    pinMode(TFT1_CS, OUTPUT); digitalWrite(TFT1_CS, HIGH);
    pinMode(TFT2_CS, OUTPUT); digitalWrite(TFT2_CS, HIGH);
    SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI);

    tft1.init(240, 320);
    tft2.init(76, 284);
    setTftSpeed(tft1, TFT1_SPI_HZ, 0);

    tft1.setRotation(1); 
    tft2.setRotation(3);
    tft2.invertDisplay(false);
    tft1.invertDisplay(false);

    tft1.fillScreen(0x0000);
    tft2.fillScreen(0x0000);

    TJpgDec.setJpgScale(1);
    TJpgDec.setSwapBytes(false);
    TJpgDec.setCallback(tft1_output);

    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder.attachHalfQuad(ENC_DT, ENC_CLK);
    encoder.setCount(0);
    pinMode(ENC_SW, INPUT_PULLUP);
    
    // Initialize USB Host Mass Storage
    msc.begin();

    logi("[setup] Starting audio task on core 0");
    BaseType_t taskRc = xTaskCreatePinnedToCore(audioTask, "AudioTask", 32768, NULL, 5, NULL, 0);
    if (taskRc != pdPASS) loge("[setup] FAILED to create AudioTask");
}

void loop() {
    dbgLoopCount++;

    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 10000) {
        lastHeartbeat = millis();
        logd("[hb] loop alive, tracks loaded: %d", (int)playlist.size());
    }

    // Toggle Play/Pause on Encoder Button Press
    static int lastButton = HIGH;
    int button = digitalRead(ENC_SW);
    if (button != lastButton) {
        if (button == LOW) {
            isPlaying = !isPlaying;
            logi("[UI] Button pressed, Playback: %s", isPlaying ? "PLAYING" : "PAUSED");
            needsUiUpdate = true;
        }
        lastButton = button;
        delay(50); // Simple debounce
    }

    // Handle Skip Tracks via Rotary Encoder
    int currentEncoderCount = encoder.getCount();
    if (abs(currentEncoderCount - lastEncoderCount) >= 1) {
        bool forward = currentEncoderCount > lastEncoderCount;
        
        std::lock_guard<std::mutex> lock(stateMutex);
        if (playlist.size() > 0) {
            if (forward) {
                currentTrackIndex = (currentTrackIndex + 1) % playlist.size();
            } else {
                currentTrackIndex = (currentTrackIndex - 1 + playlist.size()) % playlist.size();
            }
            logi("[UI] Track skipped %s to index %d", forward ? "forward" : "back", currentTrackIndex);
        }
        
        simulatedProgressMs = 0;
        isPlaying = true;
        lastEncoderCount = currentEncoderCount;
        needsUiUpdate = true;
    }

    // Process UI Redraw Requests
    if (needsUiUpdate) {
        std::string title = playlist.empty() ? "No MP3 Files Found" : std::string(playlist[currentTrackIndex].c_str());
        std::string album = playlist.empty() ? "Please insert USB" : "Local Flash Drive";
        int trackNum = playlist.empty() ? 0 : currentTrackIndex + 1;
        
        updateUI(title, "USB Audio Sink", album, trackNum, playlist.size(), simulatedProgressMs, simulatedDurationMs, isPlaying);
        needsUiUpdate = false;
    }

    // Refresh Animated UI Elements
    updateProgressBar();
    updateTextAnimations();
}

void uiTask(void* pvParameters) {
    logi("[uiTask] Started on core %d", (int)xPortGetCoreID());
    setup();
    while (true) {
        loop();
        vTaskDelay(pdMS_TO_TICKS(1)); // Yield to watchdog
    }
}

extern "C" void app_main() {
    initArduino();
    logi("[app_main] Creating UI task");
    xTaskCreatePinnedToCore(uiTask, "ui", 32768, NULL, 1, NULL, 1);
}