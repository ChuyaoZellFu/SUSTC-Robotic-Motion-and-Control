;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::I2CConfiguration)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'I2CConfiguration (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::I2CCONFIGURATION")
  (make-package "KORTEX_DRIVER::I2CCONFIGURATION"))

(in-package "ROS")
;;//! \htmlinclude I2CConfiguration.msg.html


(defclass kortex_driver::I2CConfiguration
  :super ros::object
  :slots (_device _enabled _mode _addressing ))

(defmethod kortex_driver::I2CConfiguration
  (:init
   (&key
    ((:device __device) 0)
    ((:enabled __enabled) nil)
    ((:mode __mode) 0)
    ((:addressing __addressing) 0)
    )
   (send-super :init)
   (setq _device (round __device))
   (setq _enabled __enabled)
   (setq _mode (round __mode))
   (setq _addressing (round __addressing))
   self)
  (:device
   (&optional __device)
   (if __device (setq _device __device)) _device)
  (:enabled
   (&optional (__enabled :null))
   (if (not (eq __enabled :null)) (setq _enabled __enabled)) _enabled)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:addressing
   (&optional __addressing)
   (if __addressing (setq _addressing __addressing)) _addressing)
  (:serialization-length
   ()
   (+
    ;; uint32 _device
    4
    ;; bool _enabled
    1
    ;; uint32 _mode
    4
    ;; uint32 _addressing
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _device
       (write-long _device s)
     ;; bool _enabled
       (if _enabled (write-byte -1 s) (write-byte 0 s))
     ;; uint32 _mode
       (write-long _mode s)
     ;; uint32 _addressing
       (write-long _addressing s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _device
     (setq _device (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _enabled
     (setq _enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint32 _mode
     (setq _mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _addressing
     (setq _addressing (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::I2CConfiguration :md5sum-) "97b0dc0932403bfd636c81257df2603c")
(setf (get kortex_driver::I2CConfiguration :datatype-) "kortex_driver/I2CConfiguration")
(setf (get kortex_driver::I2CConfiguration :definition-)
      "
uint32 device
bool enabled
uint32 mode
uint32 addressing
")



(provide :kortex_driver/I2CConfiguration "97b0dc0932403bfd636c81257df2603c")


