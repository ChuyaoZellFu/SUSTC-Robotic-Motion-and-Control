;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::UARTParity)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'UARTParity (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::UARTPARITY")
  (make-package "KORTEX_DRIVER::UARTPARITY"))

(in-package "ROS")
;;//! \htmlinclude UARTParity.msg.html


(intern "*UART_PARITY_UNSPECIFIED*" (find-package "KORTEX_DRIVER::UARTPARITY"))
(shadow '*UART_PARITY_UNSPECIFIED* (find-package "KORTEX_DRIVER::UARTPARITY"))
(defconstant kortex_driver::UARTParity::*UART_PARITY_UNSPECIFIED* 0)
(intern "*UART_PARITY_NONE*" (find-package "KORTEX_DRIVER::UARTPARITY"))
(shadow '*UART_PARITY_NONE* (find-package "KORTEX_DRIVER::UARTPARITY"))
(defconstant kortex_driver::UARTParity::*UART_PARITY_NONE* 1)
(intern "*UART_PARITY_ODD*" (find-package "KORTEX_DRIVER::UARTPARITY"))
(shadow '*UART_PARITY_ODD* (find-package "KORTEX_DRIVER::UARTPARITY"))
(defconstant kortex_driver::UARTParity::*UART_PARITY_ODD* 2)
(intern "*UART_PARITY_EVEN*" (find-package "KORTEX_DRIVER::UARTPARITY"))
(shadow '*UART_PARITY_EVEN* (find-package "KORTEX_DRIVER::UARTPARITY"))
(defconstant kortex_driver::UARTParity::*UART_PARITY_EVEN* 3)

(defun kortex_driver::UARTParity-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::UARTParity::*UART_PARITY_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::UARTParity::*UART_PARITY_NONE*)
        ((= const 2) 'kortex_driver::UARTParity::*UART_PARITY_ODD*)
        ((= const 3) 'kortex_driver::UARTParity::*UART_PARITY_EVEN*)
        (t nil)))

(defclass kortex_driver::UARTParity
  :super ros::object
  :slots ())

(defmethod kortex_driver::UARTParity
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

(setf (get kortex_driver::UARTParity :md5sum-) "02ea0fa72b95d9977ccc489459fb1109")
(setf (get kortex_driver::UARTParity :datatype-) "kortex_driver/UARTParity")
(setf (get kortex_driver::UARTParity :definition-)
      "
uint32 UART_PARITY_UNSPECIFIED = 0

uint32 UART_PARITY_NONE = 1

uint32 UART_PARITY_ODD = 2

uint32 UART_PARITY_EVEN = 3

")



(provide :kortex_driver/UARTParity "02ea0fa72b95d9977ccc489459fb1109")


