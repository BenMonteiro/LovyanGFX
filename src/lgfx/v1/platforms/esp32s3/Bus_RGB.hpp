/*----------------------------------------------------------------------------/
  Lovyan GFX - Graphics library for embedded devices.

Original Source:
 https://github.com/lovyan03/LovyanGFX/

Licence:
 [FreeBSD](https://github.com/lovyan03/LovyanGFX/blob/master/license.txt)

Author:
 [lovyan03](https://twitter.com/lovyan03)

Contributors:
 [ciniml](https://github.com/ciniml)
 [mongonta0716](https://github.com/mongonta0716)
 [tobozo](https://github.com/tobozo)
/----------------------------------------------------------------------------*/
#pragma once

#if __has_include (<esp_lcd_panel_rgb.h>)
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_rgb.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_interface.h>


#include <esp_private/gdma.h>
#include <hal/dma_types.h>

#include "../../Bus.hpp"
#include "../../panel/Panel_FrameBufferBase.hpp"
#include "../common.hpp"

struct lcd_cam_dev_t;
struct esp_rgb_panel_t;

namespace lgfx
{
 inline namespace v1
 {
//----------------------------------------------------------------------------

  class Bus_RGB : public IBus
  {
  public:
    struct config_t
    {
      Panel_FrameBufferBase* panel = nullptr;

      // LCD_CAM peripheral number. No need to change (only 0 for ESP32-S3.)
      int8_t port = 0;

      // pixel clock
      uint32_t freq_write = 16000000;

      int8_t pin_pclk = -1;
      int8_t pin_vsync = -1;
      int8_t pin_hsync = -1;
      int8_t pin_henable = -1;
      union
      {
        int8_t pin_data[16];
        struct
        {
          int8_t pin_d0;
          int8_t pin_d1;
          int8_t pin_d2;
          int8_t pin_d3;
          int8_t pin_d4;
          int8_t pin_d5;
          int8_t pin_d6;
          int8_t pin_d7;
          int8_t pin_d8;
          int8_t pin_d9;
          int8_t pin_d10;
          int8_t pin_d11;
          int8_t pin_d12;
          int8_t pin_d13;
          int8_t pin_d14;
          int8_t pin_d15;
        };
      };

      int8_t hsync_pulse_width = 0;
      int8_t hsync_back_porch = 0;
      int8_t hsync_front_porch = 0;
      int8_t vsync_pulse_width = 0;
      int8_t vsync_back_porch = 0;
      int8_t vsync_front_porch = 0;
      bool hsync_polarity = 0;
      bool vsync_polarity = 0;
      bool pclk_active_neg = 1;
      bool de_idle_high = 0;
      bool pclk_idle_high = 0;
    };

    const config_t& config(void) const { return _cfg; }
    void config(const config_t& config);

    bus_type_t busType(void) const override { return bus_type_t::bus_unknown; }

    bool init(void) override;
    void release(void) override;

    void beginTransaction(void) override {}
    void endTransaction(void) override {}
    void wait(void) override {}
    bool busy(void) const override { return false; }

    void flush(void) override {}
    bool writeCommand(uint32_t data, uint_fast8_t bit_length) override { return true; }
    void writeData(uint32_t data, uint_fast8_t bit_length) override {}
    void writeDataRepeat(uint32_t data, uint_fast8_t bit_length, uint32_t count) override {}
    void writePixels(pixelcopy_t* param, uint32_t length) override {}
    void writeBytes(const uint8_t* data, uint32_t length, bool dc, bool use_dma) override {}

    void initDMA(void) override {}
    void addDMAQueue(const uint8_t* data, uint32_t length) override {}
    void execDMAQueue(void) override {}
    uint8_t* getDMABuffer(uint32_t length) override;

    void beginRead(void) override {}
    void endRead(void) override {}
    uint32_t readData(uint_fast8_t bit_length) override { return 0; }
    bool readBytes(uint8_t* dst, uint32_t length, bool use_dma) override { return false; }
    void readPixels(void* dst, pixelcopy_t* param, uint32_t length) override {}

  private:
    config_t _cfg;

    dma_descriptor_t _dmadesc_restart;
    dma_descriptor_t* _dmadesc = nullptr;
    esp_lcd_i80_bus_handle_t _i80_bus = nullptr;
    int32_t _dma_ch;

    esp_lcd_panel_handle_t _panel_handle = nullptr;

    uint8_t *_frame_buffer = nullptr;
    intr_handle_t _intr_handle;
    static void lcd_default_isr_handler(void *args);

    // --- Bounce buffer (reduction de la contention bus/PSRAM avec le DMA
    // audio I2S, cf. commit dans le projet applicatif "crowpanel-bounce-buffer") ---
    // Etape 2 (instrumentation seule) : allocation + IRQ out_eof en place,
    // mais PAS ENCORE branches sur le GDMA actif (qui continue de scanner
    // l'ancien anneau complet _dmadesc, comme avant ce patch). L'ISR ne fait
    // qu'incrementer un compteur (jamais de log/allocation dans une ISR).
  public:
    // Expose temporairement pour validation materielle (etape 2) : nombre de
    // fois que l'IRQ out_eof du canal GDMA s'est declenchee depuis le demarrage.
    uint32_t debugBounceEofCount(void) const { return _bounce_eof_count; }
  private:
    // 20 lignes teste et REJETE : a introduit un artefact de "dedoublement"
    // d'image nettement pire que le defaut d'origine (pas juste inefficace,
    // activement pire). Revenu a 10 (comportement au moins aussi stable que
    // sans bounce buffer sous charge audio, meilleur sans audio - cf. les
    // deux tests precedents).
    static constexpr size_t kBounceLines = 10;
    uint8_t* _bounceA = nullptr;
    uint8_t* _bounceB = nullptr;
    dma_descriptor_t* _dmadesc_bounce_a = nullptr;
    dma_descriptor_t* _dmadesc_bounce_b = nullptr;
    dma_descriptor_t _dmadesc_bounce_restart; // etape 3 : equivalent bounce de _dmadesc_restart
    size_t _bounce_desc_count_a = 0;
    size_t _bounce_desc_count_b = 0;
    size_t _bounce_chunk_bytes = 0;   // etape 3 : octets d'un segment (kBounceLines lignes)
    uint32_t _bounce_total_chunks = 0; // etape 3 : nb de segments par image (height / kBounceLines)
    uint32_t _bounce_next_chunk = 0;   // etape 3 : prochain segment a copier depuis _frame_buffer
    bool _bounce_refill_is_a = true;   // etape 3 : le prochain refill cible A (true) ou B (false)
    intr_handle_t _bounce_intr_handle = nullptr;
    volatile uint32_t _bounce_eof_count = 0;
    // Recopie chunk0->A / chunk1->B et reinitialise l'etat d'alternance :
    // appelee une fois depuis init() (premiere image) et a chaque VSYNC
    // depuis lcd_default_isr_handler (resynchronisation de debut de trame,
    // etape 3) - IRAM_ATTR car appelable depuis une ISR.
    IRAM_ATTR void primeBounceBuffers(void);
    static void lcd_bounce_refill_isr_handler(void* args);
  };

//----------------------------------------------------------------------------
 }
}
#endif
