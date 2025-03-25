;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CustomDataSelection)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CustomDataSelection (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CUSTOMDATASELECTION")
  (make-package "KORTEX_DRIVER::CUSTOMDATASELECTION"))

(in-package "ROS")
;;//! \htmlinclude CustomDataSelection.msg.html


(defclass kortex_driver::CustomDataSelection
  :super ros::object
  :slots (_channel ))

(defmethod kortex_driver::CustomDataSelection
  (:init
   (&key
    ((:channel __channel) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _channel __channel)
   self)
  (:channel
   (&optional __channel)
   (if __channel (setq _channel __channel)) _channel)
  (:serialization-length
   ()
   (+
    ;; uint32[] _channel
    (* 4    (length _channel)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32[] _channel
     (write-long (length _channel) s)
     (dotimes (i (length _channel))
       (write-long (elt _channel i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32[] _channel
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _channel (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _channel i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get kortex_driver::CustomDataSelection :md5sum-) "308bb0056e133aee362395c22802fb41")
(setf (get kortex_driver::CustomDataSelection :datatype-) "kortex_driver/CustomDataSelection")
(setf (get kortex_driver::CustomDataSelection :definition-)
      "
uint32[] channel
")



(provide :kortex_driver/CustomDataSelection "308bb0056e133aee362395c22802fb41")


