;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::CartesianLimitationList)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'CartesianLimitationList (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CARTESIANLIMITATIONLIST")
  (make-package "KORTEX_DRIVER::CARTESIANLIMITATIONLIST"))

(in-package "ROS")
;;//! \htmlinclude CartesianLimitationList.msg.html


(defclass kortex_driver::CartesianLimitationList
  :super ros::object
  :slots (_limitations ))

(defmethod kortex_driver::CartesianLimitationList
  (:init
   (&key
    ((:limitations __limitations) ())
    )
   (send-super :init)
   (setq _limitations __limitations)
   self)
  (:limitations
   (&rest __limitations)
   (if (keywordp (car __limitations))
       (send* _limitations __limitations)
     (progn
       (if __limitations (setq _limitations (car __limitations)))
       _limitations)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/CartesianLimitation[] _limitations
    (apply #'+ (send-all _limitations :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/CartesianLimitation[] _limitations
     (write-long (length _limitations) s)
     (dolist (elem _limitations)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/CartesianLimitation[] _limitations
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _limitations (let (r) (dotimes (i n) (push (instance kortex_driver::CartesianLimitation :init) r)) r))
     (dolist (elem- _limitations)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get kortex_driver::CartesianLimitationList :md5sum-) "c96ad5571d257cbcdf15809b29c600bb")
(setf (get kortex_driver::CartesianLimitationList :datatype-) "kortex_driver/CartesianLimitationList")
(setf (get kortex_driver::CartesianLimitationList :definition-)
      "
CartesianLimitation[] limitations
================================================================================
MSG: kortex_driver/CartesianLimitation

uint32 type
float32 translation
float32 orientation
")



(provide :kortex_driver/CartesianLimitationList "c96ad5571d257cbcdf15809b29c600bb")


