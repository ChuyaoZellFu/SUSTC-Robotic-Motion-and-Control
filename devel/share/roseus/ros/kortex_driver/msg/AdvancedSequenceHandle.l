;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::AdvancedSequenceHandle)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'AdvancedSequenceHandle (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::ADVANCEDSEQUENCEHANDLE")
  (make-package "KORTEX_DRIVER::ADVANCEDSEQUENCEHANDLE"))

(in-package "ROS")
;;//! \htmlinclude AdvancedSequenceHandle.msg.html


(defclass kortex_driver::AdvancedSequenceHandle
  :super ros::object
  :slots (_handle _in_loop ))

(defmethod kortex_driver::AdvancedSequenceHandle
  (:init
   (&key
    ((:handle __handle) (instance kortex_driver::SequenceHandle :init))
    ((:in_loop __in_loop) nil)
    )
   (send-super :init)
   (setq _handle __handle)
   (setq _in_loop __in_loop)
   self)
  (:handle
   (&rest __handle)
   (if (keywordp (car __handle))
       (send* _handle __handle)
     (progn
       (if __handle (setq _handle (car __handle)))
       _handle)))
  (:in_loop
   (&optional (__in_loop :null))
   (if (not (eq __in_loop :null)) (setq _in_loop __in_loop)) _in_loop)
  (:serialization-length
   ()
   (+
    ;; kortex_driver/SequenceHandle _handle
    (send _handle :serialization-length)
    ;; bool _in_loop
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/SequenceHandle _handle
       (send _handle :serialize s)
     ;; bool _in_loop
       (if _in_loop (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/SequenceHandle _handle
     (send _handle :deserialize buf ptr-) (incf ptr- (send _handle :serialization-length))
   ;; bool _in_loop
     (setq _in_loop (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get kortex_driver::AdvancedSequenceHandle :md5sum-) "7661973c2e6febb748e9471014616c3e")
(setf (get kortex_driver::AdvancedSequenceHandle :datatype-) "kortex_driver/AdvancedSequenceHandle")
(setf (get kortex_driver::AdvancedSequenceHandle :definition-)
      "
SequenceHandle handle
bool in_loop
================================================================================
MSG: kortex_driver/SequenceHandle

uint32 identifier
uint32 permission
")



(provide :kortex_driver/AdvancedSequenceHandle "7661973c2e6febb748e9471014616c3e")


