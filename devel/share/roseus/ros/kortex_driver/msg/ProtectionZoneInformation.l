;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ProtectionZoneInformation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ProtectionZoneInformation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::PROTECTIONZONEINFORMATION")
  (make-package "KORTEX_DRIVER::PROTECTIONZONEINFORMATION"))

(in-package "ROS")
;;//! \htmlinclude ProtectionZoneInformation.msg.html


(defclass kortex_driver::ProtectionZoneInformation
  :super ros::object
  :slots (_event ))

(defmethod kortex_driver::ProtectionZoneInformation
  (:init
   (&key
    ((:event __event) 0)
    )
   (send-super :init)
   (setq _event (round __event))
   self)
  (:event
   (&optional __event)
   (if __event (setq _event __event)) _event)
  (:serialization-length
   ()
   (+
    ;; uint32 _event
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _event
       (write-long _event s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _event
     (setq _event (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::ProtectionZoneInformation :md5sum-) "85038978649328eee44c6513e8ebb7e6")
(setf (get kortex_driver::ProtectionZoneInformation :datatype-) "kortex_driver/ProtectionZoneInformation")
(setf (get kortex_driver::ProtectionZoneInformation :definition-)
      "
uint32 event
")



(provide :kortex_driver/ProtectionZoneInformation "85038978649328eee44c6513e8ebb7e6")


