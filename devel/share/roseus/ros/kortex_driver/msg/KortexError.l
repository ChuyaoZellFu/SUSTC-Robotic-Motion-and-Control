;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::KortexError)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'KortexError (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::KORTEXERROR")
  (make-package "KORTEX_DRIVER::KORTEXERROR"))

(in-package "ROS")
;;//! \htmlinclude KortexError.msg.html


(defclass kortex_driver::KortexError
  :super ros::object
  :slots (_description _code _subCode ))

(defmethod kortex_driver::KortexError
  (:init
   (&key
    ((:description __description) "")
    ((:code __code) 0)
    ((:subCode __subCode) 0)
    )
   (send-super :init)
   (setq _description (string __description))
   (setq _code (round __code))
   (setq _subCode (round __subCode))
   self)
  (:description
   (&optional __description)
   (if __description (setq _description __description)) _description)
  (:code
   (&optional __code)
   (if __code (setq _code __code)) _code)
  (:subCode
   (&optional __subCode)
   (if __subCode (setq _subCode __subCode)) _subCode)
  (:serialization-length
   ()
   (+
    ;; string _description
    4 (length _description)
    ;; uint32 _code
    4
    ;; uint32 _subCode
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _description
       (write-long (length _description) s) (princ _description s)
     ;; uint32 _code
       (write-long _code s)
     ;; uint32 _subCode
       (write-long _subCode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _description
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _description (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _code
     (setq _code (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _subCode
     (setq _subCode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::KortexError :md5sum-) "e30b4b6c4d6da8786415fb734a7dea1b")
(setf (get kortex_driver::KortexError :datatype-) "kortex_driver/KortexError")
(setf (get kortex_driver::KortexError :definition-)
      "string description
uint32 code
uint32 subCode
")



(provide :kortex_driver/KortexError "e30b4b6c4d6da8786415fb734a7dea1b")


