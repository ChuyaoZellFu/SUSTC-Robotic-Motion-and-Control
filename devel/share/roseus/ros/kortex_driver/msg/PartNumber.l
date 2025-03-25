;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::PartNumber)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'PartNumber (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::PARTNUMBER")
  (make-package "KORTEX_DRIVER::PARTNUMBER"))

(in-package "ROS")
;;//! \htmlinclude PartNumber.msg.html


(defclass kortex_driver::PartNumber
  :super ros::object
  :slots (_part_number ))

(defmethod kortex_driver::PartNumber
  (:init
   (&key
    ((:part_number __part_number) "")
    )
   (send-super :init)
   (setq _part_number (string __part_number))
   self)
  (:part_number
   (&optional __part_number)
   (if __part_number (setq _part_number __part_number)) _part_number)
  (:serialization-length
   ()
   (+
    ;; string _part_number
    4 (length _part_number)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _part_number
       (write-long (length _part_number) s) (princ _part_number s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _part_number
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _part_number (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get kortex_driver::PartNumber :md5sum-) "ce9f25be628512aac32b499f98199661")
(setf (get kortex_driver::PartNumber :datatype-) "kortex_driver/PartNumber")
(setf (get kortex_driver::PartNumber :definition-)
      "
string part_number
")



(provide :kortex_driver/PartNumber "ce9f25be628512aac32b499f98199661")


