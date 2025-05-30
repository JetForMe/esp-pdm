# MIT License

idf_component_register(
	SRCS
		"src/ESPPDM.cpp"
   INCLUDE_DIRS "src"
   REQUIRES
		esp_driver_i2s
		esp_ringbuf
)
