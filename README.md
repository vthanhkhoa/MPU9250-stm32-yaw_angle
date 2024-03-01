# MPU9250-stm32-yaw_angle
1. ERROR   
- Can't use 2 pointers to access a struct -> change pointer struct to struct variable   
```
//in struct imu_9250
MPU9250_t *pt1_p;

```
(mpu9250->pt1_p->acc_x will be error)   
We use:
```
//in struct imu_9250
MPU9250_t pt1_p;

```
(mpu9250->pt1_p->acc_x)     
- if connection isn't stable, change power to 5V (from 3V3) (maybe, or not :V)
- 
