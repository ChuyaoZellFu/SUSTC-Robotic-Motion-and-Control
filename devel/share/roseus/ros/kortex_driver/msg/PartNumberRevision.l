;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::PartNumberRevision)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'PartNumberRevision (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::PARTNUMBERREVISION")
  (make-package "KORTEX_DRIVER::PARTNUMBERREVISION"))

(in-package "ROS")
;;//! \htmlinclude PartNumberRevision.msg.html


(defclass kortex_driver::PartNumberRevision
  :super ros::object
  :slots (_part_number_revision ))

(defmethod kortex_driver::PartNumberRevision
  (:init
   (&key
    ((:part_number_revision __part_number_revision) "")
    )
   (send-super :init)
   (setq _part_number_revision (string __part_number_revision))
   self)
  (:part_number_revision
   (&optional __part_number_revision)
   (if __part_number_revision (setq _part_number_revision __part_number_revision)) _part_number_revision)
  (:serialization-length
   ()
   (+
    ;; string _part_number_revision
    4 (length _part_number_revision)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _part_number_revision
       (write-long (length _part_number_revision) s) (princ _part_number_revision s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _part_number_revision
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _part_number_revision (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get kortex_driver::PartNumberRevision :md5sum-) "4621433c3d6219edd88cd50a0ae9aa72")
(setf (get kortex_driver::PartNumberRevision :datatype-) "kortex_driver/PartNumberRevision")
(setf (get kortex_driver::PartNumberRevision :definition-)
      "
string part_number_revision
")



(provide :kortex_driver/PartNumberRevision "4621433c3d6219edd88cd50a0ae9aa72")


