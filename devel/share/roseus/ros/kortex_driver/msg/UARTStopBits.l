;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::UARTStopBits)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'UARTStopBits (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::UARTSTOPBITS")
  (make-package "KORTEX_DRIVER::UARTSTOPBITS"))

(in-package "ROS")
;;//! \htmlinclude UARTStopBits.msg.html


(intern "*UART_STOP_BITS_UNSPECIFIED*" (find-package "KORTEX_DRIVER::UARTSTOPBITS"))
(shadow '*UART_STOP_BITS_UNSPECIFIED* (find-package "KORTEX_DRIVER::UARTSTOPBITS"))
(defconstant kortex_driver::UARTStopBits::*UART_STOP_BITS_UNSPECIFIED* 0)
(intern "*UART_STOP_BITS_0_5*" (find-package "KORTEX_DRIVER::UARTSTOPBITS"))
(shadow '*UART_STOP_BITS_0_5* (find-package "KORTEX_DRIVER::UARTSTOPBITS"))
(defconstant kortex_driver::UARTStopBits::*UART_STOP_BITS_0_5* 1)
(intern "*UART_STOP_BITS_1*" (find-package "KORTEX_DRIVER::UARTSTOPBITS"))
(shadow '*UART_STOP_BITS_1* (find-package "KORTEX_DRIVER::UARTSTOPBITS"))
(defconstant kortex_driver::UARTStopBits::*UART_STOP_BITS_1* 2)
(intern "*UART_STOP_BITS_1_5*" (find-package "KORTEX_DRIVER::UARTSTOPBITS"))
(shadow '*UART_STOP_BITS_1_5* (find-package "KORTEX_DRIVER::UARTSTOPBITS"))
(defconstant kortex_driver::UARTStopBits::*UART_STOP_BITS_1_5* 3)
(intern "*UART_STOP_BITS_2*" (find-package "KORTEX_DRIVER::UARTSTOPBITS"))
(shadow '*UART_STOP_BITS_2* (find-package "KORTEX_DRIVER::UARTSTOPBITS"))
(defconstant kortex_driver::UARTStopBits::*UART_STOP_BITS_2* 4)

(defun kortex_driver::UARTStopBits-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::UARTStopBits::*UART_STOP_BITS_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::UARTStopBits::*UART_STOP_BITS_0_5*)
        ((= const 2) 'kortex_driver::UARTStopBits::*UART_STOP_BITS_1*)
        ((= const 3) 'kortex_driver::UARTStopBits::*UART_STOP_BITS_1_5*)
        ((= const 4) 'kortex_driver::UARTStopBits::*UART_STOP_BITS_2*)
        (t nil)))

(defclass kortex_driver::UARTStopBits
  :super ros::object
  :slots ())

(defmethod kortex_driver::UARTStopBits
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

(setf (get kortex_driver::UARTStopBits :md5sum-) "41247c97dd0da3882638e6dea902eead")
(setf (get kortex_driver::UARTStopBits :datatype-) "kortex_driver/UARTStopBits")
(setf (get kortex_driver::UARTStopBits :definition-)
      "
uint32 UART_STOP_BITS_UNSPECIFIED = 0

uint32 UART_STOP_BITS_0_5 = 1

uint32 UART_STOP_BITS_1 = 2

uint32 UART_STOP_BITS_1_5 = 3

uint32 UART_STOP_BITS_2 = 4

")



(provide :kortex_driver/UARTStopBits "41247c97dd0da3882638e6dea902eead")


