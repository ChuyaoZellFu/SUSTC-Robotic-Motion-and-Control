;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::LoopSelection)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'LoopSelection (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::LOOPSELECTION")
  (make-package "KORTEX_DRIVER::LOOPSELECTION"))

(in-package "ROS")
;;//! \htmlinclude LoopSelection.msg.html


(defclass kortex_driver::LoopSelection
  :super ros::object
  :slots (_loop_selection ))

(defmethod kortex_driver::LoopSelection
  (:init
   (&key
    ((:loop_selection __loop_selection) 0)
    )
   (send-super :init)
   (setq _loop_selection (round __loop_selection))
   self)
  (:loop_selection
   (&optional __loop_selection)
   (if __loop_selection (setq _loop_selection __loop_selection)) _loop_selection)
  (:serialization-length
   ()
   (+
    ;; uint32 _loop_selection
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _loop_selection
       (write-long _loop_selection s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _loop_selection
     (setq _loop_selection (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::LoopSelection :md5sum-) "404b340f5699ef6e44d5690bdce228f3")
(setf (get kortex_driver::LoopSelection :datatype-) "kortex_driver/LoopSelection")
(setf (get kortex_driver::LoopSelection :definition-)
      "
uint32 loop_selection
")



(provide :kortex_driver/LoopSelection "404b340f5699ef6e44d5690bdce228f3")


