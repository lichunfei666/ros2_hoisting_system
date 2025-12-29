/**
 ********************************************************************
 * @file    hal_uart.c
 * @brief
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <errno.h>
#include <dji_logger.h>
#include "hal_uart.h"
#include <asm-generic/termbits-common.h>
#include <time.h>

/* Private constants ---------------------------------------------------------*/
#define UART_DEV_NAME_STR_SIZE             (128)
#define DJI_SYSTEM_CMD_STR_MAX_SIZE        (64)
#define DJI_SYSTEM_RESULT_STR_MAX_SIZE     (128)

/* Private types -------------------------------------------------------------*/
typedef struct {
    int uartFd;
} T_UartHandleStruct;

/* Private values -------------------------------------------------------------*/

/* Private functions declaration ---------------------------------------------*/
static char *find_usb_serial_device(uint16_t vid, uint16_t pid);

/* Exported functions definition ---------------------------------------------*/
T_DjiReturnCode HalUart_Init(E_DjiHalUartNum uartNum, uint32_t baudRate, T_DjiUartHandle *uartHandle)
{
    T_UartHandleStruct *uartHandleStruct;
    struct termios options;
    struct flock lock;
    T_DjiReturnCode returnCode = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    char uartName[UART_DEV_NAME_STR_SIZE];
    
    USER_LOG_INFO("HalUart_Init called with uartNum=%d, baudRate=%u", uartNum, baudRate);

    uartHandleStruct = malloc(sizeof(T_UartHandleStruct));
    if (uartHandleStruct == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_MEMORY_ALLOC_FAILED;
    }

    // Find USB serial device
    char *device_path = NULL;
    
    USER_LOG_INFO("USE_FIXED_UART_DEVICE is set to: %d", USE_FIXED_UART_DEVICE);
    
    #if USE_FIXED_UART_DEVICE
        // Use fixed device path
        device_path = strdup(FIXED_UART_DEVICE_PATH);
        USER_LOG_INFO("Using fixed UART device path: %s", FIXED_UART_DEVICE_PATH);
    #else
        // Find by VID and PID
        USER_LOG_INFO("Trying to find USB serial device with VID:0x%04X, PID:0x%04X", 
                     USB_UART_CONNECTED_TO_UAV_VID, USB_UART_CONNECTED_TO_UAV_PID);
        device_path = find_usb_serial_device(USB_UART_CONNECTED_TO_UAV_VID, USB_UART_CONNECTED_TO_UAV_PID);
        if (device_path == NULL) {
            USER_LOG_ERROR("Can't find USB serial device with VID:0x%04X, PID:0x%04X", 
                          USB_UART_CONNECTED_TO_UAV_VID, USB_UART_CONNECTED_TO_UAV_PID);
            goto free_uart_handle;
        } else {
            USER_LOG_INFO("Successfully found USB serial device: %s", device_path);
        }
    #endif
    
    strcpy(uartName, device_path);
    free(device_path);

    // Check if has permission to operate the device
    if (access(uartName, R_OK | W_OK) != 0) {
        USER_LOG_ERROR("Can't operate the device. "
                       "Probably the device has no operation permission. "
                       "Please execute command 'sudo chmod 777 %s' to add permission. ", uartName);
        goto free_uart_handle;
    }

    uartHandleStruct->uartFd = open(uartName, (unsigned) O_RDWR | (unsigned) O_NOCTTY | (unsigned) O_NDELAY);
    if (uartHandleStruct->uartFd == -1) {
        goto free_uart_handle;
    }

    // Forbid multiple psdk programs to access the serial port
    lock.l_type = F_WRLCK;
    lock.l_pid = getpid();
    lock.l_whence = SEEK_SET;
    lock.l_start = 0;
    lock.l_len = 0;

    if (fcntl(uartHandleStruct->uartFd, F_GETLK, &lock) < 0) {
        goto close_uart_fd;
    }
    if (lock.l_type != F_UNLCK) {
        goto close_uart_fd;
    }
    lock.l_type = F_WRLCK;
    lock.l_pid = getpid();
    lock.l_whence = SEEK_SET;
    lock.l_start = 0;
    lock.l_len = 0;
    if (fcntl(uartHandleStruct->uartFd, F_SETLKW, &lock) < 0) {
        goto close_uart_fd;
    }

    if (tcgetattr(uartHandleStruct->uartFd, &options) != 0) {
        goto close_uart_fd;
    }

    switch (baudRate) {
        case 115200:
            cfsetispeed(&options, B115200);
            cfsetospeed(&options, B115200);
            break;
        case 230400:
            cfsetispeed(&options, B230400);
            cfsetospeed(&options, B230400);
            break;
        case 460800:
            cfsetispeed(&options, B460800);
            cfsetospeed(&options, B460800);
            break;
        case 921600:
            cfsetispeed(&options, B921600);
            cfsetospeed(&options, B921600);
            break;
        case 1000000:
            cfsetispeed(&options, B1000000);
            cfsetospeed(&options, B1000000);
            break;
        default:
            goto close_uart_fd;
    }

    options.c_cflag |= (unsigned) CLOCAL;
    options.c_cflag |= (unsigned) CREAD;
    options.c_cflag &= ~(unsigned) CRTSCTS;
    options.c_cflag &= ~(unsigned) CSIZE;
    options.c_cflag |= (unsigned) CS8;
    options.c_cflag &= ~(unsigned) PARENB;
    options.c_iflag &= ~(unsigned) INPCK;
    options.c_cflag &= ~(unsigned) CSTOPB;
    options.c_oflag &= ~(unsigned) OPOST;
    options.c_lflag &= ~((unsigned) ICANON | (unsigned) ECHO | (unsigned) ECHOE | (unsigned) ISIG);
    options.c_iflag &= ~((unsigned) BRKINT | (unsigned) ICRNL | (unsigned) INPCK | (unsigned) ISTRIP | (unsigned) IXON);
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;

    tcflush(uartHandleStruct->uartFd, TCIFLUSH);

    if (tcsetattr(uartHandleStruct->uartFd, TCSANOW, &options) != 0) {
        goto close_uart_fd;
    }

    *uartHandle = uartHandleStruct;

    return returnCode;

close_uart_fd:
    close(uartHandleStruct->uartFd);

free_uart_handle:
    free(uartHandleStruct);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
}

T_DjiReturnCode HalUart_DeInit(T_DjiUartHandle uartHandle)
{
    int32_t ret;
    T_UartHandleStruct *uartHandleStruct = (T_UartHandleStruct *) uartHandle;

    if (uartHandle == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    ret = close(uartHandleStruct->uartFd);
    if (ret < 0) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    free(uartHandleStruct);

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode HalUart_WriteData(T_DjiUartHandle uartHandle, const uint8_t *buf, uint32_t len, uint32_t *realLen)
{
    int32_t ret;
    T_UartHandleStruct *uartHandleStruct = (T_UartHandleStruct *) uartHandle;

    if (uartHandle == NULL || buf == NULL || len == 0 || realLen == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    // 添加静态变量，用于周期性统计和显示总写入数据
    static uint64_t total_write_bytes = 0;
    static time_t last_print_time = 0;
    
    ret = write(uartHandleStruct->uartFd, buf, len);
    if (ret >= 0) {
        *realLen = ret;
        total_write_bytes += ret;
        
        // 周期性（每3秒）打印一次总写入字节数
        time_t current_time = time(NULL);
        if (current_time - last_print_time >= 3) {
            USER_LOG_INFO("UART Total Written: %lu bytes", total_write_bytes);
            last_print_time = current_time;
        }
    } else {
        USER_LOG_ERROR("UART Write Error: %d", errno);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode HalUart_ReadData(T_DjiUartHandle uartHandle, uint8_t *buf, uint32_t len, uint32_t *realLen)
{
    int32_t ret;
    T_UartHandleStruct *uartHandleStruct = (T_UartHandleStruct *) uartHandle;

    if (uartHandle == NULL || buf == NULL || len == 0 || realLen == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    ret = read(uartHandleStruct->uartFd, buf, len);
    if (ret >= 0) {
        *realLen = ret;
        // 添加调试信息 - 3秒周期打印读数据总数
        static time_t last_log_time = 0;
        static uint32_t total_bytes_read = 0;
        
        // 更新计数器
        total_bytes_read += ret;
        
        // 获取当前时间
        time_t current_time = time(NULL);
        
        // 检查是否需要打印日志（3秒周期）
        if (current_time - last_log_time >= 3) {
            USER_LOG_INFO("UART Total Read bytes: %d", total_bytes_read);
            
            // 更新上次打印时间
            last_log_time = current_time;
        }
    } else {
        USER_LOG_ERROR("UART Read Error: %d", errno);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode HalUart_GetStatus(E_DjiHalUartNum uartNum, T_DjiUartStatus *status)
{
    if (uartNum == DJI_HAL_UART_NUM_0) {
        status->isConnect = true;
    } else if (uartNum == DJI_HAL_UART_NUM_1) {
        status->isConnect = true;
    } else {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode HalUart_GetDeviceInfo(T_DjiHalUartDeviceInfo *deviceInfo)
{
    if (deviceInfo == NULL) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    deviceInfo->vid = USB_UART_CONNECTED_TO_UAV_VID;
    deviceInfo->pid = USB_UART_CONNECTED_TO_UAV_PID;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

/* Private functions definition-----------------------------------------------*/
/**
 * @brief Find USB serial device by VID and PID
 * @param vid Vendor ID
 * @param pid Product ID
 * @return Device path if found, NULL otherwise
 */
static char *find_usb_serial_device(uint16_t vid, uint16_t pid)
{
    struct udev *udev;
    struct udev_enumerate *enumerate;
    struct udev_list_entry *devices, *dev_list_entry;
    struct udev_device *dev;
    char *device_path = NULL;
    char vid_str[5];
    char pid_str[5];
    
    // Convert VID and PID to hex strings
    sprintf(vid_str, "%04x", vid);
    sprintf(pid_str, "%04x", pid);
    
    // Create udev context
    udev = udev_new();
    if (!udev) {
        USER_LOG_ERROR("Can't create udev context");
        return NULL;
    }
    
    // Create enumerate context
    enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, "tty");
    udev_enumerate_scan_devices(enumerate);
    
    // Get list of devices
    devices = udev_enumerate_get_list_entry(enumerate);
    udev_list_entry_foreach(dev_list_entry, devices) {
        const char *path;
        
        // Get device path
        path = udev_list_entry_get_name(dev_list_entry);
        dev = udev_device_new_from_syspath(udev, path);
        
        // Check if device has parent (USB device)
        const struct udev_device *parent = udev_device_get_parent_with_subsystem_devtype(
            dev, "usb", "usb_device");
        if (parent) {
            // Get VID and PID
            const char *dev_vid = udev_device_get_sysattr_value((struct udev_device*)parent, "idVendor");
            const char *dev_pid = udev_device_get_sysattr_value((struct udev_device*)parent, "idProduct");
            
            // Compare VID and PID
            if (dev_vid && dev_pid) {
                if (strcmp(dev_vid, vid_str) == 0 && strcmp(dev_pid, pid_str) == 0) {
                    // Found matching device
                    const char *dev_node = udev_device_get_devnode(dev);
                    if (dev_node) {
                        device_path = strdup(dev_node);
                        USER_LOG_INFO("Found USB serial device: %s, VID:0x%s, PID:0x%s", 
                                     dev_node, dev_vid, dev_pid);
                        break;
                    }
                }
            }
        }
        
        udev_device_unref(dev);
    }
    
    // Cleanup
    udev_enumerate_unref(enumerate);
    udev_unref(udev);
    
    return device_path;
}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/