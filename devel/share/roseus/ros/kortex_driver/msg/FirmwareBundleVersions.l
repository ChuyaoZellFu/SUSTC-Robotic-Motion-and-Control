;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::FirmwareBundleVersions)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'FirmwareBundleVersions (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::FIRMWAREBUNDLEVERSIONS")
  (make-package "KORTEX_DRIVER::FIRMWAREBUNDLEVERSIONS"))

(in-package "ROS")
;;//! \htmlinclude FirmwareBundleVersions.msg.html


(defclass kortex_driver::FirmwareBundleVersions
  :super ros::object
  :slots (_main_bundle_version _components_versions ))

(defmethod kortex_driver::FirmwareBundleVersions
  (:init
   (&key
    ((:main_bundle_version __main_bundle_version) "")
    ((:components_versions __components_versions) ())
    )
   (send-super :init)
   (setq _main_bundle_version (string __main_bundle_version))
   (setq _components_versions __components_versions)
   self)
  (:main_bundle_version
   (&optional __main_bundle_version)
   (if __main_bundle_version (setq _main_bundle_version __main_bundle_version)) _main_bundle_version)
  (:components_versions
   (&rest __components_versions)
   (if (keywordp (car __components_versions))
       (send* _components_versions __components_versions)
     (progn
       (if __components_versions (setq _components_versions (car __components_versions)))
       _components_versions)))
  (:serialization-length
   ()
   (+
    ;; string _main_bundle_version
    4 (length _main_bundle_version)
    ;; kortex_driver/FirmwareComponentVersion[] _components_versions
    (apply #'+ (send-all _components_versions :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _main_bundle_version
       (write-long (length _main_bundle_version) s) (princ _main_bundle_version s)
     ;; kortex_driver/FirmwareComponentVersion[] _components_versions
     (write-long (length _components_versions) s)
     (dolist (elem _components_versions)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _main_bundle_version
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _main_bundle_version (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; kortex_driver/FirmwareComponentVersion[] _components_versions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _components_versions (let (r) (dotimes (i n) (push (instance kortex_driver::FirmwareComponentVersion :init) r)) r))
     (dolist (elem- _components_versions)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::FirmwareBundleVersions :md5sum-) "c4f9de789ae8af0bd7b668a4f8f7ed49")
(setf (get kortex_driver::FirmwareBundleVersions :datatype-) "kortex_driver/FirmwareBundleVersions")
(setf (get kortex_driver::FirmwareBundleVersions :definition-)
      "
string main_bundle_version
FirmwareComponentVersion[] components_versions
================================================================================
MSG: kortex_driver/FirmwareComponentVersion

string name
string version
uint32 device_id
")



(provide :kortex_driver/FirmwareBundleVersions "c4f9de789ae8af0bd7b668a4f8f7ed49")


