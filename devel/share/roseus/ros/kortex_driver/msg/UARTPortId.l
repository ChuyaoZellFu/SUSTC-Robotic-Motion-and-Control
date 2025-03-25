;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::UARTPortId)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'UARTPortId (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::UARTPORTID")
  (make-package "KORTEX_DRIVER::UARTPORTID"))

(in-package "ROS")
;;//! \htmlinclude UARTPortId.msg.html


(intern "*UART_PORT_UNSPECIFIED*" (find-package "KORTEX_DRIVER::UARTPORTID"))
(shadow '*UART_PORT_UNSPECIFIED* (find-package "KORTEX_DRIVER::UARTPORTID"))
(defconstant kortex_driver::UARTPortId::*UART_PORT_UNSPECIFIED* 0)
(intern "*UART_PORT_EXPANSION*" (find-package "KORTEX_DRIVER::UARTPORTID"))
(shadow '*UART_PORT_EXPANSION* (find-package "KORTEX_DRIVER::UARTPORTID"))
(defconstant kortex_driver::UARTPortId::*UART_PORT_EXPANSION* 1)

(defun kortex_driver::UARTPortId-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::UARTPortId::*UART_PORT_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::UARTPortId::*UART_PORT_EXPANSION*)
        (t nil)))

(defclass kortex_driver::UARTPortId
  :super ros::object
  :slots ())

(defmethod kortex_driver::UARTPortId
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

(setf (get kortex_driver::UARTPortId :md5sum-) "9298e5854915673894eb15fed41f2130")
(setf (get kortex_driver::UARTPortId :datatype-) "kortex_driver/UARTPortId")
(setf (get kortex_driver::UARTPortId :definition-)
      "
uint32 UART_PORT_UNSPECIFIED = 0

uint32 UART_PORT_EXPANSION = 1

")



(provide :kortex_driver/UARTPortId "9298e5854915673894eb15fed41f2130")


