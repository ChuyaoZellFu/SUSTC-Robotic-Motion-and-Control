; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude RunMode.msg.html

(cl:defclass <RunMode> (roslisp-msg-protocol:ros-message)
  ((run_mode
    :reader run_mode
    :initarg :run_mode
    :type cl:integer
    :initform 0))
)

(cl:defclass RunMode (<RunMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RunMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RunMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<RunMode> is deprecated: use kortex_driver-msg:RunMode instead.")))

(cl:ensure-generic-function 'run_mode-val :lambda-list '(m))
(cl:defmethod run_mode-val ((m <RunMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:run_mode-val is deprecated.  Use kortex_driver-msg:run_mode instead.")
  (run_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RunMode>) ostream)
  "Serializes a message object of type '<RunMode>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'run_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'run_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'run_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'run_mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RunMode>) istream)
  "Deserializes a message object of type '<RunMode>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'run_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'run_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'run_mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'run_mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RunMode>)))
  "Returns string type for a message object of type '<RunMode>"
  "kortex_driver/RunMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunMode)))
  "Returns string type for a message object of type 'RunMode"
  "kortex_driver/RunMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RunMode>)))
  "Returns md5sum for a message object of type '<RunMode>"
  "abe2eb774be1b28bf7c30a06ea5e3691")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RunMode)))
  "Returns md5sum for a message object of type 'RunMode"
  "abe2eb774be1b28bf7c30a06ea5e3691")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RunMode>)))
  "Returns full string definition for message of type '<RunMode>"
  (cl:format cl:nil "~%uint32 run_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RunMode)))
  "Returns full string definition for message of type 'RunMode"
  (cl:format cl:nil "~%uint32 run_mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RunMode>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RunMode>))
  "Converts a ROS message object to a list"
  (cl:list 'RunMode
    (cl:cons ':run_mode (run_mode msg))
))
