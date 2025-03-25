;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::BluetoothEnableState)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'BluetoothEnableState (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BLUETOOTHENABLESTATE")
  (make-package "KORTEX_DRIVER::BLUETOOTHENABLESTATE"))

(in-package "ROS")
;;//! \htmlinclude BluetoothEnableState.msg.html


(defclass kortex_driver::BluetoothEnableState
  :super ros::object
  :slots (_enabled ))

(defmethod kortex_driver::BluetoothEnableState
  (:init
   (&key
    ((:enabled __enabled) nil)
    )
   (send-super :init)
   (setq _enabled __enabled)
   self)
  (:enabled
   (&optional (__enabled :null))
   (if (not (eq __enabled :null)) (setq _enabled __enabled)) _enabled)
  (:serialization-length
   ()
   (+
    ;; bool _enabled
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _enabled
       (if _enabled (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _enabled
     (setq _enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get kortex_driver::BluetoothEnableState :md5sum-) "2815464f55ab63684cc1bc38072d0b9b")
(setf (get kortex_driver::BluetoothEnableState :datatype-) "kortex_driver/BluetoothEnableState")
(setf (get kortex_driver::BluetoothEnableState :definition-)
      "
bool enabled
")



(provide :kortex_driver/BluetoothEnableState "2815464f55ab63684cc1bc38072d0b9b")


