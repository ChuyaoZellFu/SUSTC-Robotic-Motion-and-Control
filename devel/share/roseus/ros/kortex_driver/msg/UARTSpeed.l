;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::UARTSpeed)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'UARTSpeed (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::UARTSPEED")
  (make-package "KORTEX_DRIVER::UARTSPEED"))

(in-package "ROS")
;;//! \htmlinclude UARTSpeed.msg.html


(intern "*UART_SPEED_UNSPECIFIED*" (find-package "KORTEX_DRIVER::UARTSPEED"))
(shadow '*UART_SPEED_UNSPECIFIED* (find-package "KORTEX_DRIVER::UARTSPEED"))
(defconstant kortex_driver::UARTSpeed::*UART_SPEED_UNSPECIFIED* 0)
(intern "*UART_SPEED_4800*" (find-package "KORTEX_DRIVER::UARTSPEED"))
(shadow '*UART_SPEED_4800* (find-package "KORTEX_DRIVER::UARTSPEED"))
(defconstant kortex_driver::UARTSpeed::*UART_SPEED_4800* 1)
(intern "*UART_SPEED_9600*" (find-package "KORTEX_DRIVER::UARTSPEED"))
(shadow '*UART_SPEED_9600* (find-package "KORTEX_DRIVER::UARTSPEED"))
(defconstant kortex_driver::UARTSpeed::*UART_SPEED_9600* 2)
(intern "*UART_SPEED_19200*" (find-package "KORTEX_DRIVER::UARTSPEED"))
(shadow '*UART_SPEED_19200* (find-package "KORTEX_DRIVER::UARTSPEED"))
(defconstant kortex_driver::UARTSpeed::*UART_SPEED_19200* 3)
(intern "*UART_SPEED_38400*" (find-package "KORTEX_DRIVER::UARTSPEED"))
(shadow '*UART_SPEED_38400* (find-package "KORTEX_DRIVER::UARTSPEED"))
(defconstant kortex_driver::UARTSpeed::*UART_SPEED_38400* 4)
(intern "*UART_SPEED_57600*" (find-package "KORTEX_DRIVER::UARTSPEED"))
(shadow '*UART_SPEED_57600* (find-package "KORTEX_DRIVER::UARTSPEED"))
(defconstant kortex_driver::UARTSpeed::*UART_SPEED_57600* 5)
(intern "*UART_SPEED_115200*" (find-package "KORTEX_DRIVER::UARTSPEED"))
(shadow '*UART_SPEED_115200* (find-package "KORTEX_DRIVER::UARTSPEED"))
(defconstant kortex_driver::UARTSpeed::*UART_SPEED_115200* 6)
(intern "*UART_SPEED_230400*" (find-package "KORTEX_DRIVER::UARTSPEED"))
(shadow '*UART_SPEED_230400* (find-package "KORTEX_DRIVER::UARTSPEED"))
(defconstant kortex_driver::UARTSpeed::*UART_SPEED_230400* 7)
(intern "*UART_SPEED_460800*" (find-package "KORTEX_DRIVER::UARTSPEED"))
(shadow '*UART_SPEED_460800* (find-package "KORTEX_DRIVER::UARTSPEED"))
(defconstant kortex_driver::UARTSpeed::*UART_SPEED_460800* 8)
(intern "*UART_SPEED_921600*" (find-package "KORTEX_DRIVER::UARTSPEED"))
(shadow '*UART_SPEED_921600* (find-package "KORTEX_DRIVER::UARTSPEED"))
(defconstant kortex_driver::UARTSpeed::*UART_SPEED_921600* 9)
(intern "*UART_SPEED_1382400*" (find-package "KORTEX_DRIVER::UARTSPEED"))
(shadow '*UART_SPEED_1382400* (find-package "KORTEX_DRIVER::UARTSPEED"))
(defconstant kortex_driver::UARTSpeed::*UART_SPEED_1382400* 10)
(intern "*UART_SPEED_1612800*" (find-package "KORTEX_DRIVER::UARTSPEED"))
(shadow '*UART_SPEED_1612800* (find-package "KORTEX_DRIVER::UARTSPEED"))
(defconstant kortex_driver::UARTSpeed::*UART_SPEED_1612800* 11)
(intern "*UART_SPEED_1843200*" (find-package "KORTEX_DRIVER::UARTSPEED"))
(shadow '*UART_SPEED_1843200* (find-package "KORTEX_DRIVER::UARTSPEED"))
(defconstant kortex_driver::UARTSpeed::*UART_SPEED_1843200* 12)
(intern "*UART_SPEED_2073600*" (find-package "KORTEX_DRIVER::UARTSPEED"))
(shadow '*UART_SPEED_2073600* (find-package "KORTEX_DRIVER::UARTSPEED"))
(defconstant kortex_driver::UARTSpeed::*UART_SPEED_2073600* 13)
(intern "*UART_SPEED_2188800*" (find-package "KORTEX_DRIVER::UARTSPEED"))
(shadow '*UART_SPEED_2188800* (find-package "KORTEX_DRIVER::UARTSPEED"))
(defconstant kortex_driver::UARTSpeed::*UART_SPEED_2188800* 14)
(intern "*UART_SPEED_2246400*" (find-package "KORTEX_DRIVER::UARTSPEED"))
(shadow '*UART_SPEED_2246400* (find-package "KORTEX_DRIVER::UARTSPEED"))
(defconstant kortex_driver::UARTSpeed::*UART_SPEED_2246400* 15)

(defun kortex_driver::UARTSpeed-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::UARTSpeed::*UART_SPEED_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::UARTSpeed::*UART_SPEED_4800*)
        ((= const 2) 'kortex_driver::UARTSpeed::*UART_SPEED_9600*)
        ((= const 3) 'kortex_driver::UARTSpeed::*UART_SPEED_19200*)
        ((= const 4) 'kortex_driver::UARTSpeed::*UART_SPEED_38400*)
        ((= const 5) 'kortex_driver::UARTSpeed::*UART_SPEED_57600*)
        ((= const 6) 'kortex_driver::UARTSpeed::*UART_SPEED_115200*)
        ((= const 7) 'kortex_driver::UARTSpeed::*UART_SPEED_230400*)
        ((= const 8) 'kortex_driver::UARTSpeed::*UART_SPEED_460800*)
        ((= const 9) 'kortex_driver::UARTSpeed::*UART_SPEED_921600*)
        ((= const 10) 'kortex_driver::UARTSpeed::*UART_SPEED_1382400*)
        ((= const 11) 'kortex_driver::UARTSpeed::*UART_SPEED_1612800*)
        ((= const 12) 'kortex_driver::UARTSpeed::*UART_SPEED_1843200*)
        ((= const 13) 'kortex_driver::UARTSpeed::*UART_SPEED_2073600*)
        ((= const 14) 'kortex_driver::UARTSpeed::*UART_SPEED_2188800*)
        ((= const 15) 'kortex_driver::UARTSpeed::*UART_SPEED_2246400*)
        (t nil)))

(defclass kortex_driver::UARTSpeed
  :super ros::object
  :slots ())

(defmethod kortex_driver::UARTSpeed
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get kortex_driver::UARTSpeed :md5sum-) "75fb3218a247fa4c93b3a9a066077af5")
(setf (get kortex_driver::UARTSpeed :datatype-) "kortex_driver/UARTSpeed")
(setf (get kortex_driver::UARTSpeed :definition-)
      "
uint32 UART_SPEED_UNSPECIFIED = 0

uint32 UART_SPEED_4800 = 1

uint32 UART_SPEED_9600 = 2

uint32 UART_SPEED_19200 = 3

uint32 UART_SPEED_38400 = 4

uint32 UART_SPEED_57600 = 5

uint32 UART_SPEED_115200 = 6

uint32 UART_SPEED_230400 = 7

uint32 UART_SPEED_460800 = 8

uint32 UART_SPEED_921600 = 9

uint32 UART_SPEED_1382400 = 10

uint32 UART_SPEED_1612800 = 11

uint32 UART_SPEED_1843200 = 12

uint32 UART_SPEED_2073600 = 13

uint32 UART_SPEED_2188800 = 14

uint32 UART_SPEED_2246400 = 15

")



(provide :kortex_driver/UARTSpeed "75fb3218a247fa4c93b3a9a066077af5")


