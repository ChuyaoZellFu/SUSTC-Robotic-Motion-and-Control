;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SafetyEnable)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SafetyEnable (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SAFETYENABLE")
  (make-package "KORTEX_DRIVER::SAFETYENABLE"))

(in-package "ROS")
;;//! \htmlinclude SafetyEnable.msg.html


(defclass kortex_driver::SafetyEnable
  :super ros::object
  :slots (_handle _enable ))

(defmethod kortex_driver::SafetyEnable
  (:init
   (&key
    ((:handle __handle) (instance kortex_driver::SafetyHandle :init))
    ((:enable __enable) nil)
    )
   (send-super :init)
   (setq _handle __handle)
   (setq _enable __enable)
   self)
  (:handle
   (&rest __handle)
   (if (keywordp (car __handle))
       (send* _handle __handle)
     (progn
       (if __handle (setq _handle (car __handle)))
       _handle)))
  (:enable
   (&optional (__enable :null))
   (if (not (eq __enable :null)) (setq _enable __enable)) _enable)
  (:serialization-length
   ()
   (+
    ;; kortex_driver/SafetyHandle _handle
    (send _handle :serialization-length)
    ;; bool _enable
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SafetyHandle _handle
       (send _handle :serialize s)
     ;; bool _enable
       (if _enable (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SafetyHandle _handle
     (send _handle :deserialize buf ptr-) (incf ptr- (send _handle :serialization-length))
   ;; bool _enable
     (setq _enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get kortex_driver::SafetyEnable :md5sum-) "5f3f3341f0f54081acc9ca67a9de861a")
(setf (get kortex_driver::SafetyEnable :datatype-) "kortex_driver/SafetyEnable")
(setf (get kortex_driver::SafetyEnable :definition-)
      "
SafetyHandle handle
bool enable
================================================================================
MSG: kortex_driver/SafetyHandle

uint32 identifier
")



(provide :kortex_driver/SafetyEnable "5f3f3341f0f54081acc9ca67a9de861a")


