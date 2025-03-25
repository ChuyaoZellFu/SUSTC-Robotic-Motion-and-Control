;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::PayloadInformation)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'PayloadInformation (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::PAYLOADINFORMATION")
  (make-package "KORTEX_DRIVER::PAYLOADINFORMATION"))

(in-package "ROS")
;;//! \htmlinclude PayloadInformation.msg.html


(defclass kortex_driver::PayloadInformation
  :super ros::object
  :slots (_payload_mass _payload_mass_center ))

(defmethod kortex_driver::PayloadInformation
  (:init
   (&key
    ((:payload_mass __payload_mass) 0.0)
    ((:payload_mass_center __payload_mass_center) (instance kortex_driver::ControlConfig_Position :init))
    )
   (send-super :init)
   (setq _payload_mass (float __payload_mass))
   (setq _payload_mass_center __payload_mass_center)
   self)
  (:payload_mass
   (&optional __payload_mass)
   (if __payload_mass (setq _payload_mass __payload_mass)) _payload_mass)
  (:payload_mass_center
   (&rest __payload_mass_center)
   (if (keywordp (car __payload_mass_center))
       (send* _payload_mass_center __payload_mass_center)
     (progn
       (if __payload_mass_center (setq _payload_mass_center (car __payload_mass_center)))
       _payload_mass_center)))
  (:serialization-length
   ()
   (+
    ;; float32 _payload_mass
    4
    ;; kortex_driver/ControlConfig_Position _payload_mass_center
    (send _payload_mass_center :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _payload_mass
       (sys::poke _payload_mass (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; kortex_driver/ControlConfig_Position _payload_mass_center
       (send _payload_mass_center :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _payload_mass
     (setq _payload_mass (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; kortex_driver/ControlConfig_Position _payload_mass_center
     (send _payload_mass_center :deserialize buf ptr-) (incf ptr- (send _payload_mass_center :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::PayloadInformation :md5sum-) "b6414681a38d62f0a2b6ff0a6a813f53")
(setf (get kortex_driver::PayloadInformation :datatype-) "kortex_driver/PayloadInformation")
(setf (get kortex_driver::PayloadInformation :definition-)
      "
float32 payload_mass
ControlConfig_Position payload_mass_center
================================================================================
MSG: kortex_driver/ControlConfig_Position

float32 x
float32 y
float32 z
")



(provide :kortex_driver/PayloadInformation "b6414681a38d62f0a2b6ff0a6a813f53")


