#define LED_CONFIG_GREEN			\
  {						\
    .mode = BLE_UIS_LED_MODE_CONST,		\
      .data =					\
      {						\
        .mode_const =				\
        {					\
	  .r  = 10,				\
	  .g  = 177,				\
	  .b  = 10,				\
        }					\
      }						\
  }

#define LED_CONFIG_PURPLE			\
  {						\
    .mode = BLE_UIS_LED_MODE_CONST,		\
      .data =					\
      {						\
        .mode_const =				\
        {					\
	  .r  = 170,				\
	  .g  = 20,				\
	  .b  = 20,				\
        }					\
      }						\
  }

#define LED_CONFIG_RED				\
  {						\
    .mode = BLE_UIS_LED_MODE_CONST,		\
      .data =					\
      {						\
        .mode_const =				\
        {					\
	  .r  = 177,				\
	  .g  = 0,				\
	  .b  = 0,				\
        }					\
      }						\
  }

#define LED_CONFIG_BLUE				\
  {						\
    .mode = BLE_UIS_LED_MODE_CONST,		\
      .data =					\
      {						\
        .mode_const =				\
        {					\
	  .r  = 10,				\
	  .g  = 10,				\
	  .b  = 177,				\
        }					\
      }						\
  }

#define LED_CONFIG_WHITE				\
  {						\
    .mode = BLE_UIS_LED_MODE_BREATHE,		\
      .data =					\
      {						\
        .mode_const =				\
        {					\
	  .r  = 255,				\
	  .g  = 255,				\
	  .b  = 255,				\
        }					\
      }						\
  }

#define LED_CONFIG_YELLOW				\
  {						\
    .mode = BLE_UIS_LED_MODE_CONST,		\
      .data =					\
      {						\
        .mode_const =				\
        {					\
	  .r  = 255,				\
	  .g  = 191,				\
	  .b  = 0,				\
        }					\
      }						\
  }
