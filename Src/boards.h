#define DEFAULT               0
#define ENTREE                1

#if BOARD_ID==ENTREE
#define IOPIN_LED0    GPIO_PIN_13
#define IOPIN_PORT    GPIOC
// default
#else
#define IOPIN_LED0    GPIO_PIN_13
#define IOPIN_PORT    GPIOC
#endif
