;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetFirmwareVersion)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetFirmwareVersion (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETFIRMWAREVERSION")
  (make-package "KORTEX_DRIVER::GETFIRMWAREVERSION"))
(unless (find-package "KORTEX_DRIVER::GETFIRMWAREVERSIONREQUEST")
  (make-package "KORTEX_DRIVER::GETFIRMWAREVERSIONREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETFIRMWAREVERSIONRESPONSE")
  (make-package "KORTEX_DRIVER::GETFIRMWAREVERSIONRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetFirmwareVersionRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetFirmwareVersionRequest
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

(defclass kortex_driver::GetFirmwareVersionResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetFirmwareVersionResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::FirmwareVersion :init))
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
    ;; kortex_driver/FirmwareVersion _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/FirmwareVersion _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/FirmwareVersion _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetFirmwareVersion
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetFirmwareVersion :md5sum-) "b9c18ab692345d79a149debba533958f")
(setf (get kortex_driver::GetFirmwareVersion :datatype-) "kortex_driver/GetFirmwareVersion")
(setf (get kortex_driver::GetFirmwareVersion :request) kortex_driver::GetFirmwareVersionRequest)
(setf (get kortex_driver::GetFirmwareVersion :response) kortex_driver::GetFirmwareVersionResponse)

(defmethod kortex_driver::GetFirmwareVersionRequest
  (:response () (instance kortex_driver::GetFirmwareVersionResponse :init)))

(setf (get kortex_driver::GetFirmwareVersionRequest :md5sum-) "b9c18ab692345d79a149debba533958f")
(setf (get kortex_driver::GetFirmwareVersionRequest :datatype-) "kortex_driver/GetFirmwareVersionRequest")
(setf (get kortex_driver::GetFirmwareVersionRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
FirmwareVersion output

================================================================================
MSG: kortex_driver/FirmwareVersion

")

(setf (get kortex_driver::GetFirmwareVersionResponse :md5sum-) "b9c18ab692345d79a149debba533958f")
(setf (get kortex_driver::GetFirmwareVersionResponse :datatype-) "kortex_driver/GetFirmwareVersionResponse")
(setf (get kortex_driver::GetFirmwareVersionResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
FirmwareVersion output

================================================================================
MSG: kortex_driver/FirmwareVersion

")



(provide :kortex_driver/GetFirmwareVersion "b9c18ab692345d79a149debba533958f")


