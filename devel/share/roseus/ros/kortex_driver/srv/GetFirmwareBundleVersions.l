;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::GetFirmwareBundleVersions)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'GetFirmwareBundleVersions (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::GETFIRMWAREBUNDLEVERSIONS")
  (make-package "KORTEX_DRIVER::GETFIRMWAREBUNDLEVERSIONS"))
(unless (find-package "KORTEX_DRIVER::GETFIRMWAREBUNDLEVERSIONSREQUEST")
  (make-package "KORTEX_DRIVER::GETFIRMWAREBUNDLEVERSIONSREQUEST"))
(unless (find-package "KORTEX_DRIVER::GETFIRMWAREBUNDLEVERSIONSRESPONSE")
  (make-package "KORTEX_DRIVER::GETFIRMWAREBUNDLEVERSIONSRESPONSE"))

(in-package "ROS")





(defclass kortex_driver::GetFirmwareBundleVersionsRequest
  :super ros::object
  :slots (_input ))

(defmethod kortex_driver::GetFirmwareBundleVersionsRequest
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

(defclass kortex_driver::GetFirmwareBundleVersionsResponse
  :super ros::object
  :slots (_output ))

(defmethod kortex_driver::GetFirmwareBundleVersionsResponse
  (:init
   (&key
    ((:output __output) (instance kortex_driver::FirmwareBundleVersions :init))
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
    ;; kortex_driver/FirmwareBundleVersions _output
    (send _output :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/FirmwareBundleVersions _output
       (send _output :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/FirmwareBundleVersions _output
     (send _output :deserialize buf ptr-) (incf ptr- (send _output :serialization-length))
   ;;
   self)
  )

(defclass kortex_driver::GetFirmwareBundleVersions
  :super ros::object
  :slots ())

(setf (get kortex_driver::GetFirmwareBundleVersions :md5sum-) "8a198c1bd1965cc2d1e06f5a3548e004")
(setf (get kortex_driver::GetFirmwareBundleVersions :datatype-) "kortex_driver/GetFirmwareBundleVersions")
(setf (get kortex_driver::GetFirmwareBundleVersions :request) kortex_driver::GetFirmwareBundleVersionsRequest)
(setf (get kortex_driver::GetFirmwareBundleVersions :response) kortex_driver::GetFirmwareBundleVersionsResponse)

(defmethod kortex_driver::GetFirmwareBundleVersionsRequest
  (:response () (instance kortex_driver::GetFirmwareBundleVersionsResponse :init)))

(setf (get kortex_driver::GetFirmwareBundleVersionsRequest :md5sum-) "8a198c1bd1965cc2d1e06f5a3548e004")
(setf (get kortex_driver::GetFirmwareBundleVersionsRequest :datatype-) "kortex_driver/GetFirmwareBundleVersionsRequest")
(setf (get kortex_driver::GetFirmwareBundleVersionsRequest :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
FirmwareBundleVersions output

================================================================================
MSG: kortex_driver/FirmwareBundleVersions

string main_bundle_version
FirmwareComponentVersion[] components_versions
================================================================================
MSG: kortex_driver/FirmwareComponentVersion

string name
string version
")

(setf (get kortex_driver::GetFirmwareBundleVersionsResponse :md5sum-) "8a198c1bd1965cc2d1e06f5a3548e004")
(setf (get kortex_driver::GetFirmwareBundleVersionsResponse :datatype-) "kortex_driver/GetFirmwareBundleVersionsResponse")
(setf (get kortex_driver::GetFirmwareBundleVersionsResponse :definition-)
      "Empty input

================================================================================
MSG: kortex_driver/Empty
---
FirmwareBundleVersions output

================================================================================
MSG: kortex_driver/FirmwareBundleVersions

string main_bundle_version
FirmwareComponentVersion[] components_versions
================================================================================
MSG: kortex_driver/FirmwareComponentVersion

string name
string version
")



(provide :kortex_driver/GetFirmwareBundleVersions "8a198c1bd1965cc2d1e06f5a3548e004")


