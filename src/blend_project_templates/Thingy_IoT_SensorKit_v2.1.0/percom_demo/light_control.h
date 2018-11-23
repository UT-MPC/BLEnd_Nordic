#define LED_CONFIG_GREEN			\
  {						\
    .mode = BLE_UIS_LED_MODE_CONST,		\
      .data =					\
      {						\
        .mode_const =				\
        {					\
	  .r  = 11,				\
	  .g  = 102,				\
	  .b  = 35				\
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
	  .r  = 75,				\
	  .g  = 0,				\
	  .b  = 130				\
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
	  .b  = 0				\
        }					\
      }						\
  }

#define LED_CONFIG_WHITE				\
  {						\
    .mode = BLE_UIS_LED_MODE_CONST,		\
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
