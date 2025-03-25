;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::UARTWordLength)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'UARTWordLength (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::UARTWORDLENGTH")
  (make-package "KORTEX_DRIVER::UARTWORDLENGTH"))

(in-package "ROS")
;;//! \htmlinclude UARTWordLength.msg.html


(intern "*UART_WORD_LENGTH_UNSPECIFIED*" (find-package "KORTEX_DRIVER::UARTWORDLENGTH"))
(shadow '*UART_WORD_LENGTH_UNSPECIFIED* (find-package "KORTEX_DRIVER::UARTWORDLENGTH"))
(defconstant kortex_driver::UARTWordLength::*UART_WORD_LENGTH_UNSPECIFIED* 0)
(intern "*UART_WORD_LENGTH_7*" (find-package "KORTEX_DRIVER::UARTWORDLENGTH"))
(shadow '*UART_WORD_LENGTH_7* (find-package "KORTEX_DRIVER::UARTWORDLENGTH"))
(defconstant kortex_driver::UARTWordLength::*UART_WORD_LENGTH_7* 1)
(intern "*UART_WORD_LENGTH_8*" (find-package "KORTEX_DRIVER::UARTWORDLENGTH"))
(shadow '*UART_WORD_LENGTH_8* (find-package "KORTEX_DRIVER::UARTWORDLENGTH"))
(defconstant kortex_driver::UARTWordLength::*UART_WORD_LENGTH_8* 2)
(intern "*UART_WORD_LENGTH_9*" (find-package "KORTEX_DRIVER::UARTWORDLENGTH"))
(shadow '*UART_WORD_LENGTH_9* (find-package "KORTEX_DRIVER::UARTWORDLENGTH"))
(defconstant kortex_driver::UARTWordLength::*UART_WORD_LENGTH_9* 3)

(defun kortex_driver::UARTWordLength-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::UARTWordLength::*UART_WORD_LENGTH_UNSPECIFIED*)
        ((= const 1) 'kortex_driver::UARTWordLength::*UART_WORD_LENGTH_7*)
        ((= const 2) 'kortex_driver::UARTWordLength::*UART_WORD_LENGTH_8*)
        ((= const 3) 'kortex_driver::UARTWordLength::*UART_WORD_LENGTH_9*)
        (t nil)))

(defclass kortex_driver::UARTWordLength
  :super ros::object
  :slots ())

(defmethod kortex_driver::UARTWordLength
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

(setf (get kortex_driver::UARTWordLength :md5sum-) "d77bc1cf43de7d8c64548008ccbf4e86")
(setf (get kortex_driver::UARTWordLength :datatype-) "kortex_driver/UARTWordLength")
(setf (get kortex_driver::UARTWordLength :definition-)
      "
uint32 UART_WORD_LENGTH_UNSPECIFIED = 0

uint32 UART_WORD_LENGTH_7 = 1

uint32 UART_WORD_LENGTH_8 = 2

uint32 UART_WORD_LENGTH_9 = 3

")



(provide :kortex_driver/UARTWordLength "d77bc1cf43de7d8c64548008ccbf4e86")


