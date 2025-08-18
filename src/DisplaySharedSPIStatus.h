/*
 * Singleton class designed to share SPI status between
 * multiple TFT display instances.
 * 
 * There needs to be a copy of this header in every display
 * driver library, but they MUST be identical
 */
#if !defined(_DISPLAY_SHARED_SPI_STATUS_H_)
#define _DISPLAY_SHARED_SPI_STATUS_H_

class DisplaySharedSPIStatus 
{
    static const int NUM_SPI = 3;
    DisplaySharedSPIStatus() {} // Constructor is private
  public:
    static DisplaySharedSPIStatus& getInstance()
    {
        static DisplaySharedSPIStatus instance; // Guaranteed to be destroyed.
                              // Instantiated on first use.
        return instance;
    }

    // Delete the methods we don't want.
    DisplaySharedSPIStatus(DisplaySharedSPIStatus const&) = delete;
    void operator=(DisplaySharedSPIStatus const&)  = delete;

    // SPI port status to be shared across instances
    struct status_s 
    {
        bool _begin_done = false;
        volatile uint8_t _dma_state = 0;
        uint8_t _pending_rx_count = 0;
        uint32_t _spi_tcr_current = 0; 
        DMAChannel DMAch{false};
    } status[NUM_SPI];
    status_s& operator[](uint8_t idx) 
        { return status[idx]; }
};
#endif // !defined(_DISPLAY_SHARED_SPI_STATUS_H_)

