;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CartesianReferenceFrameInfo)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CartesianReferenceFrameInfo (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CARTESIANREFERENCEFRAMEINFO")
  (make-package "KORTEX_DRIVER::CARTESIANREFERENCEFRAMEINFO"))

(in-package "ROS")
;;//! \htmlinclude CartesianReferenceFrameInfo.msg.html


(defclass kortex_driver::CartesianReferenceFrameInfo
  :super ros::object
  :slots (_reference_frame ))

(defmethod kortex_driver::CartesianReferenceFrameInfo
  (:init
   (&key
    ((:reference_frame __reference_frame) 0)
    )
   (send-super :init)
   (setq _reference_frame (round __reference_frame))
   self)
  (:reference_frame
   (&optional __reference_frame)
   (if __reference_frame (setq _reference_frame __reference_frame)) _reference_frame)
  (:serialization-length
   ()
   (+
    ;; uint32 _reference_frame
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _reference_frame
       (write-long _reference_frame s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _reference_frame
     (setq _reference_frame (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::CartesianReferenceFrameInfo :md5sum-) "c6e1a124b6ab31faa150246edd7907de")
(setf (get kortex_driver::CartesianReferenceFrameInfo :datatype-) "kortex_driver/CartesianReferenceFrameInfo")
(setf (get kortex_driver::CartesianReferenceFrameInfo :definition-)
      "
uint32 reference_frame
")



(provide :kortex_driver/CartesianReferenceFrameInfo "c6e1a124b6ab31faa150246edd7907de")


