idf_component_register(SRCS "main.c" "./UserDevices/User_Client.c" "./UserDevices/User_Uart.c" "./UserDevices/TMCx300.c" "./Application/App_Unittest.c" "./UserDevices/bsp_as5600.c"
                    INCLUDE_DIRS "." "./UserDevices" "./Application")
