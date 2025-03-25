;; Auto-generated. Do not edit!


(when (boundp 'kortex_driver::ControllerElementHandle)
  (if (not (find-package "KORTEX_DRIVER"))
    (make-package "KORTEX_DRIVER"))
  (shadow 'ControllerElementHandle (find-package "KORTEX_DRIVER")))
(unless (find-package "KORTEX_DRIVER::CONTROLLERELEMENTHANDLE")
  (make-package "KORTEX_DRIVER::CONTROLLERELEMENTHANDLE"))

(in-package "ROS")
;;//! \htmlinclude ControllerElementHandle.msg.html


(defclass kortex_driver::ControllerElementHandle
  :super ros::object
  :slots (_controller_handle _oneof_identifier ))

(defmethod kortex_driver::ControllerElementHandle
  (:init
   (&key
    ((:controller_handle __controller_handle) (instance kortex_driver::ControllerHandle :init))
    ((:oneof_identifier __oneof_identifier) (instance kortex_driver::ControllerElementHandle_identifier :init))
    )
   (send-super :init)
   (setq _controller_handle __controller_handle)
   (setq _oneof_identifier __oneof_identifier)
   self)
  (:controller_handle
   (&rest __controller_handle)
   (if (keywordp (car __controller_handle))
       (send* _controller_handle __controller_handle)
     (progn
       (if __controller_handle (setq _controller_handle (car __controller_handle)))
       _controller_handle)))
  (:oneof_identifier
   (&rest __oneof_identifier)
   (if (keywordp (car __oneof_identifier))
       (send* _oneof_identifier __oneof_identifier)
     (progn
       (if __oneof_identifier (setq _oneof_identifier (car __oneof_identifier)))
       _oneof_identifier)))
  (:serialization-length
   ()
   (+
    ;; kortex_driver/ControllerHandle _controller_handle
    (send _controller_handle :serialization-length)
    ;; kortex_driver/ControllerElementHandle_identifier _oneof_identifier
    (send _oneof_identifier :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; kortex_driver/ControllerHandle _controller_handle
       (send _controller_handle :serialize s)
     ;; kortex_driver/ControllerElementHandle_identifier _oneof_identifier
       (send _oneof_identifier :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; kortex_driver/ControllerHandle _controller_handle
     (send _controller_handle :deserialize buf ptr-) (incf ptr- (send _controller_handle :serialization-length))
   ;; kortex_driver/ControllerElementHandle_identifier _oneof_identifier
     (send _oneof_identifier :deserialize buf ptr-) (incf ptr- (send _oneof_identifier :serialization-length))
   ;;
   self)
  )

(setf (get kortex_driver::ControllerElementHandle :md5sum-) "8c89bd366e7348e92cb61f477829e5ba")
(setf (get kortex_driver::ControllerElementHandle :datatype-) "kortex_driver/ControllerElementHandle")
(setf (get kortex_driver::ControllerElementHandle :definition-)
      "
ControllerHandle controller_handle
ControllerElementHandle_identifier oneof_identifier
================================================================================
MSG: kortex_driver/ControllerHandle

uint32 type
uint32 controller_identifier
================================================================================
MSG: kortex_driver/ControllerElementHandle_identifier

uint32[] button
uint32[] axis
")



(provide :kortex_driver/ControllerElementHandle "8c89bd366e7348e92cb61f477829e5ba")


