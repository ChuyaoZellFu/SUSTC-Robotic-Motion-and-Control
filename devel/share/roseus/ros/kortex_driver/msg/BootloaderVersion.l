;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::BootloaderVersion)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'BootloaderVersion (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::BOOTLOADERVERSION")
  (make-package "KORTEX_DRIVER::BOOTLOADERVERSION"))

(in-package "ROS")
;;//! \htmlinclude BootloaderVersion.msg.html


(defclass kortex_driver::BootloaderVersion
  :super ros::object
  :slots (_bootloader_version ))

(defmethod kortex_driver::BootloaderVersion
  (:init
   (&key
    ((:bootloader_version __bootloader_version) 0)
    )
   (send-super :init)
   (setq _bootloader_version (round __bootloader_version))
   self)
  (:bootloader_version
   (&optional __bootloader_version)
   (if __bootloader_version (setq _bootloader_version __bootloader_version)) _bootloader_version)
  (:serialization-length
   ()
   (+
    ;; uint32 _bootloader_version
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _bootloader_version
       (write-long _bootloader_version s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _bootloader_version
     (setq _bootloader_version (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::BootloaderVersion :md5sum-) "ec0c056f9a04b0432415c72e0d38271e")
(setf (get kortex_driver::BootloaderVersion :datatype-) "kortex_driver/BootloaderVersion")
(setf (get kortex_driver::BootloaderVersion :definition-)
      "
uint32 bootloader_version
")



(provide :kortex_driver/BootloaderVersion "ec0c056f9a04b0432415c72e0d38271e")


