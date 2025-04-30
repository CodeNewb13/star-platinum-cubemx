define watch_mpu
  echo Starting MPU monitor...\n
  while 1
    printf "accel_x = %d\taccel_y = %d\taccel_z = %d\n", accel_x, accel_y, accel_z
    shell sleep 0.3
  end
end

watch_mpu

