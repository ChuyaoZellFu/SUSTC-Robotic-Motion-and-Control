;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::PowerOnSelfTestResult)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'PowerOnSelfTestResult (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::POWERONSELFTESTRESULT")
  (make-package "KORTEX_DRIVER::POWERONSELFTESTRESULT"))

(in-package "ROS")
;;//! \htmlinclude PowerOnSelfTestResult.msg.html


(defclass kortex_driver::PowerOnSelfTestResult
  :super ros::object
  :slots (_power_on_self_test_result ))

(defmethod kortex_driver::PowerOnSelfTestResult
  (:init
   (&key
    ((:power_on_self_test_result __power_on_self_test_result) 0)
    )
   (send-super :init)
   (setq _power_on_self_test_result (round __power_on_self_test_result))
   self)
  (:power_on_self_test_result
   (&optional __power_on_self_test_result)
   (if __power_on_self_test_result (setq _power_on_self_test_result __power_on_self_test_result)) _power_on_self_test_result)
  (:serialization-length
   ()
   (+
    ;; uint32 _power_on_self_test_result
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _power_on_self_test_result
       (write-long _power_on_self_test_result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _power_on_self_test_result
     (setq _power_on_self_test_result (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get kortex_driver::PowerOnSelfTestResult :md5sum-) "927efe338b454b142f376d1936670eb0")
(setf (get kortex_driver::PowerOnSelfTestResult :datatype-) "kortex_driver/PowerOnSelfTestResult")
(setf (get kortex_driver::PowerOnSelfTestResult :definition-)
      "
uint32 power_on_self_test_result
")



(provide :kortex_driver/PowerOnSelfTestResult "927efe338b454b142f376d1936670eb0")


