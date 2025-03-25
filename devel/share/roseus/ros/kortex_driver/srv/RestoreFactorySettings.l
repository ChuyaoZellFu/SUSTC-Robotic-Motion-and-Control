;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::RestoreFactorySettings)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'RestoreFactorySettings (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::RESTOREFACTORYSETTINGS")
  (make-package "KORTEX_DRIVER::RESTOREFACTORYSETTINGS"))
(unless (find-package "KORTEX_DRIVER::RESTOREFACTORYSETTINGSREQUEST")
  (make-package "KORTEX_DRIVER::RESTOREFACTORYSETTINGSREQUEST"))
(unless (find-package "KORTEX_DRIVER::RESTOREFACTORYSETTINGSRESPONSE")
  (make-package "KORTEX_DRIVER::RESTOREFACTORYSETTINGSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::RestoreFactorySettingsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::RestoreFactorySettingsRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::Empty :init))
    )
   (send-super :init)
   (setq _input __input)
   self)
  (:input
   (&rest __input)
   (if (keywordp (car __input))
       (send* _input __input)
     (progn
       (if __input (setq _input (car __input)))
       _input)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Empty _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::RestoreFactorySettingsResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::RestoreFactorySettingsResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::Empty :init))
    )
   (send-super :init)
   (setq _output __output)
   self)
  (:output
   (&rest __output)
   (if (keywordp (car __output))
       (send* _output __output)
     (progn
       (if __output (setq _output (car __output)))
       _output)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/Empty _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/Empty _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/Empty _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::RestoreFactorySettings
  :super ros::object
  :slots ())

(setf (get kortex_driver::RestoreFactorySettings :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::RestoreFactorySettings :datatype-) "kortex_driver/RestoreFactorySettings")
(setf (get kortex_driver::RestoreFactorySettings :request) kortex_driver::RestoreFactorySettingsRequest)
(setf (get kortex_driver::RestoreFactorySettings :response) kortex_driver::RestoreFactorySettingsResponse)

(defmethod kortex_driver::RestoreFactorySettingsRequest
  (:response () (instance kortex_driver::RestoreFactorySettingsResponse :init)))

(setf (get kortex_driver::RestoreFactorySettingsRequest :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::RestoreFactorySettingsRequest :datatype-) "kortex_driver/RestoreFactorySettingsRequest")
(setf (get kortex_driver::RestoreFactorySettingsRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::RestoreFactorySettingsResponse :md5sum-) "f335b819dc59099fe3124b36f140ad07")
(setf (get kortex_driver::RestoreFactorySettingsResponse :datatype-) "kortex_driver/RestoreFactorySettingsResponse")
(setf (get kortex_driver::RestoreFactorySettingsResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/RestoreFactorySettings "f335b819dc59099fe3124b36f140ad07")


