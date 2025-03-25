;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::DeleteProtectionZone)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'DeleteProtectionZone (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::DELETEPROTECTIONZONE")
  (make-package "KORTEX_DRIVER::DELETEPROTECTIONZONE"))
(unless (find-package "KORTEX_DRIVER::DELETEPROTECTIONZONEREQUEST")
  (make-package "KORTEX_DRIVER::DELETEPROTECTIONZONEREQUEST"))
(unless (find-package "KORTEX_DRIVER::DELETEPROTECTIONZONERESPONSE")
  (make-package "KORTEX_DRIVER::DELETEPROTECTIONZONERESPONSE"))

(in-package "ROS")





(defclass kortex_driver::DeleteProtectionZoneRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::DeleteProtectionZoneRequest
  (:init
   (&key
    ((:input __input) (instance kortex_driver::ProtectionZoneHandle :init))
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
    ;; kortex_driver/ProtectionZoneHandle _input
    (send _input :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ProtectionZoneHandle _input
       (send _input :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ProtectionZoneHandle _input
     (send _input :deserialize buf ptr-) (incf ptr- (send _input :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::DeleteProtectionZoneResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::DeleteProtectionZoneResponse
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

(defclass kortex_driver::DeleteProtectionZone
  :super ros::object
  :slots ())

(setf (get kortex_driver::DeleteProtectionZone :md5sum-) "04dfaeca45772f660e0913aa84774e13")
(setf (get kortex_driver::DeleteProtectionZone :datatype-) "kortex_driver/DeleteProtectionZone")
(setf (get kortex_driver::DeleteProtectionZone :request) kortex_driver::DeleteProtectionZoneRequest)
(setf (get kortex_driver::DeleteProtectionZone :response) kortex_driver::DeleteProtectionZoneResponse)

(defmethod kortex_driver::DeleteProtectionZoneRequest
  (:response () (instance kortex_driver::DeleteProtectionZoneResponse :init)))

(setf (get kortex_driver::DeleteProtectionZoneRequest :md5sum-) "04dfaeca45772f660e0913aa84774e13")
(setf (get kortex_driver::DeleteProtectionZoneRequest :datatype-) "kortex_driver/DeleteProtectionZoneRequest")
(setf (get kortex_driver::DeleteProtectionZoneRequest :definition-)
      "ProtectionZoneHandle input

================================================================================
MSG: kortex_driver/ProtectionZoneHandle

uint32 identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")

(setf (get kortex_driver::DeleteProtectionZoneResponse :md5sum-) "04dfaeca45772f660e0913aa84774e13")
(setf (get kortex_driver::DeleteProtectionZoneResponse :datatype-) "kortex_driver/DeleteProtectionZoneResponse")
(setf (get kortex_driver::DeleteProtectionZoneResponse :definition-)
      "ProtectionZoneHandle input

================================================================================
MSG: kortex_driver/ProtectionZoneHandle

uint32 identifier
---
Empty output

================================================================================
MSG: kortex_driver/Empty
")



(provide :kortex_driver/DeleteProtectionZone "04dfaeca45772f660e0913aa84774e13")


