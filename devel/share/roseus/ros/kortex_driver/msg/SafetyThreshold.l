;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SafetyThreshold)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SafetyThreshold (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SAFETYTHRESHOLD")
  (make-package "KORTEX_DRIVER::SAFETYTHRESHOLD"))

(in-package "ROS")
;;//! \htmlinclude SafetyThreshold.msg.html


(defclass kortex_driver::SafetyThreshold
  :super ros::object
  :slots (_handle _value ))

(defmethod kortex_driver::SafetyThreshold
  (:init
   (&key
    ((:handle __handle) (instance kortex_driver::SafetyHandle :init))
    ((:value __value) 0.0)
    )
   (send-super :init)
   (setq _handle __handle)
   (setq _value (float __value))
   self)
  (:handle
   (&rest __handle)
   (if (keywordp (car __handle))
       (send* _handle __handle)
     (progn
       (if __handle (setq _handle (car __handle)))
       _handle)))
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; kortex_driver/SafetyHandle _handle
    (send _handle :serialization-length)
    ;; float32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SafetyHandle _handle
       (send _handle :serialize s)
     ;; float32 _value
       (sys::poke _value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SafetyHandle _handle
     (send _handle :deserialize buf ptr-) (incf ptr- (send _handle :serialization-length))
   ;; float32 _value
     (setq _value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::SafetyThreshold :md5sum-) "6785d727fa30955d28c049a2ab6d6c82")
(setf (get kortex_driver::SafetyThreshold :datatype-) "kortex_driver/SafetyThreshold")
(setf (get kortex_driver::SafetyThreshold :definition-)
      "
SafetyHandle handle
float32 value
================================================================================
MSG: kortex_driver/SafetyHandle

uint32 identifier
")



(provide :kortex_driver/SafetyThreshold "6785d727fa30955d28c049a2ab6d6c82")


