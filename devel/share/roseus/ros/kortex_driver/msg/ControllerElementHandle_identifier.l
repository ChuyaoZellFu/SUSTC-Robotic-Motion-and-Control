;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControllerElementHandle_identifier)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControllerElementHandle_identifier (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLLERELEMENTHANDLE_IDENTIFIER")
  (make-package "KORTEX_DRIVER::CONTROLLERELEMENTHANDLE_IDENTIFIER"))

(in-package "ROS")
;;//! \htmlinclude ControllerElementHandle_identifier.msg.html


(defclass kortex_driver::ControllerElementHandle_identifier
  :super ros::object
  :slots (_button _axis ))

(defmethod kortex_driver::ControllerElementHandle_identifier
  (:init
   (&key
    ((:button __button) (make-array 0 :initial-element 0 :element-type :integer))
    ((:axis __axis) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _button __button)
   (setq _axis __axis)
   self)
  (:button
   (&optional __button)
   (if __button (setq _button __button)) _button)
  (:axis
   (&optional __axis)
   (if __axis (setq _axis __axis)) _axis)
  (:serialization-length
   ()
   (+
    ;; uint32[] _button
    (* 4    (length _button)) 4
    ;; uint32[] _axis
    (* 4    (length _axis)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32[] _button
     (write-long (length _button) s)
     (dotimes (i (length _button))
       (write-long (elt _button i) s)
       )
     ;; uint32[] _axis
     (write-long (length _axis) s)
     (dotimes (i (length _axis))
       (write-long (elt _axis i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32[] _button
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _button (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _button i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; uint32[] _axis
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _axis (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _axis i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get kortex_driver::ControllerElementHandle_identifier :md5sum-) "41dfd5b8a1f2350cdbaae4a32c67f201")
(setf (get kortex_driver::ControllerElementHandle_identifier :datatype-) "kortex_driver/ControllerElementHandle_identifier")
(setf (get kortex_driver::ControllerElementHandle_identifier :definition-)
      "
uint32[] button
uint32[] axis
")



(provide :kortex_driver/ControllerElementHandle_identifier "41dfd5b8a1f2350cdbaae4a32c67f201")


