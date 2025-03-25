;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SafetyEvent)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SafetyEvent (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SAFETYEVENT")
  (make-package "KORTEX_DRIVER::SAFETYEVENT"))

(in-package "ROS")
;;//! \htmlinclude SafetyEvent.msg.html


(defclass kortex_driver::SafetyEvent
  :super ros::object
  :slots (_safety_handle ))

(defmethod kortex_driver::SafetyEvent
  (:init
   (&key
    ((:safety_handle __safety_handle) (instance kortex_driver::SafetyHandle :init))
    )
   (send-super :init)
   (setq _safety_handle __safety_handle)
   self)
  (:safety_handle
   (&rest __safety_handle)
   (if (keywordp (car __safety_handle))
       (send* _safety_handle __safety_handle)
     (progn
       (if __safety_handle (setq _safety_handle (car __safety_handle)))
       _safety_handle)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/SafetyHandle _safety_handle
    (send _safety_handle :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SafetyHandle _safety_handle
       (send _safety_handle :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SafetyHandle _safety_handle
     (send _safety_handle :deserialize buf ptr-) (incf ptr- (send _safety_handle :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::SafetyEvent :md5sum-) "48c102b68919349f9a9bb31c7e295748")
(setf (get kortex_driver::SafetyEvent :datatype-) "kortex_driver/SafetyEvent")
(setf (get kortex_driver::SafetyEvent :definition-)
      "
SafetyHandle safety_handle
================================================================================
MSG: kortex_driver/SafetyHandle

uint32 identifier
")



(provide :kortex_driver/SafetyEvent "48c102b68919349f9a9bb31c7e295748")


