;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::FirmwareComponentVersion)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'FirmwareComponentVersion (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::FIRMWARECOMPONENTVERSION")
  (make-package "KORTEX_DRIVER::FIRMWARECOMPONENTVERSION"))

(in-package "ROS")
;;//! \htmlinclude FirmwareComponentVersion.msg.html


(defclass kortex_driver::FirmwareComponentVersion
  :super ros::object
  :slots (_name _version _device_id ))

(defmethod kortex_driver::FirmwareComponentVersion
  (:init
   (&key
    ((:name __name) "")
    ((:version __version) "")
    ((:device_id __device_id) 0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _version (string __version))
   (setq _device_id (round __device_id))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:version
   (&optional __version)
   (if __version (setq _version __version)) _version)
  (:device_id
   (&optional __device_id)
   (if __device_id (setq _device_id __device_id)) _device_id)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; string _version
    4 (length _version)
    ;; uint32 _device_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _version
       (write-long (length _version) s) (princ _version s)
     ;; uint32 _device_id
       (write-long _device_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _version
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _version (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _device_id
     (setq _device_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::FirmwareComponentVersion :md5sum-) "33679c8f8b46a7fffb7ad238111a28bb")
(setf (get kortex_driver::FirmwareComponentVersion :datatype-) "kortex_driver/FirmwareComponentVersion")
(setf (get kortex_driver::FirmwareComponentVersion :definition-)
      "
string name
string version
uint32 device_id
")



(provide :kortex_driver/FirmwareComponentVersion "33679c8f8b46a7fffb7ad238111a28bb")


