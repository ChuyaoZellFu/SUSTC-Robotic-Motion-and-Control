;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::SoundType)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'SoundType (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SOUNDTYPE")
  (make-package "KORTEX_DRIVER::SOUNDTYPE"))

(in-package "ROS")
;;//! \htmlinclude SoundType.msg.html


(intern "*UNSPECIFIED_SOUND_TYPE*" (find-package "KORTEX_DRIVER::SOUNDTYPE"))
(shadow '*UNSPECIFIED_SOUND_TYPE* (find-package "KORTEX_DRIVER::SOUNDTYPE"))
(defconstant kortex_driver::SoundType::*UNSPECIFIED_SOUND_TYPE* 0)
(intern "*BIP_SERIES*" (find-package "KORTEX_DRIVER::SOUNDTYPE"))
(shadow '*BIP_SERIES* (find-package "KORTEX_DRIVER::SOUNDTYPE"))
(defconstant kortex_driver::SoundType::*BIP_SERIES* 1)
(intern "*SINGLE_BIP*" (find-package "KORTEX_DRIVER::SOUNDTYPE"))
(shadow '*SINGLE_BIP* (find-package "KORTEX_DRIVER::SOUNDTYPE"))
(defconstant kortex_driver::SoundType::*SINGLE_BIP* 2)

(defun kortex_driver::SoundType-to-symbol (const)
  (cond
        ((= const 0) 'kortex_driver::SoundType::*UNSPECIFIED_SOUND_TYPE*)
        ((= const 1) 'kortex_driver::SoundType::*BIP_SERIES*)
        ((= const 2) 'kortex_driver::SoundType::*SINGLE_BIP*)
        (t nil)))

(defclass kortex_driver::SoundType
  :super ros::object
  :slots ())

(defmethod kortex_driver::SoundType
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get kortex_driver::SoundType :md5sum-) "478cf44e3f93e67de58f0a6007a206fc")
(setf (get kortex_driver::SoundType :datatype-) "kortex_driver/SoundType")
(setf (get kortex_driver::SoundType :definition-)
      "
uint32 UNSPECIFIED_SOUND_TYPE = 0

uint32 BIP_SERIES = 1

uint32 SINGLE_BIP = 2

")



(provide :kortex_driver/SoundType "478cf44e3f93e67de58f0a6007a206fc")


