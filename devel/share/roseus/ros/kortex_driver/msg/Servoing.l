;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::Servoing)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'Servoing (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::SERVOING")
  (make-package "KORTEX_DRIVER::SERVOING"))

(in-package "ROS")
;;//! \htmlinclude Servoing.msg.html


(defclass kortex_driver::Servoing
  :super ros::object
  :slots (_enabled ))

(defmethod kortex_driver::Servoing
  (:init
   (&key
    ((:enabled __enabled) nil)
    )
   (send-super :init)
   (setq _enabled __enabled)
   self)
  (:enabled
   (&optional (__enabled :null))
   (if (not (eq __enabled :null)) (setq _enabled __enabled)) _enabled)
  (:serialization-length
   ()
   (+
    ;; bool _enabled
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _enabled
       (if _enabled (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _enabled
     (setq _enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get kortex_driver::Servoing :md5sum-) "2815464f55ab63684cc1bc38072d0b9b")
(setf (get kortex_driver::Servoing :datatype-) "kortex_driver/Servoing")
(setf (get kortex_driver::Servoing :definition-)
      "
bool enabled
")



(provide :kortex_driver/Servoing "2815464f55ab63684cc1bc38072d0b9b")


