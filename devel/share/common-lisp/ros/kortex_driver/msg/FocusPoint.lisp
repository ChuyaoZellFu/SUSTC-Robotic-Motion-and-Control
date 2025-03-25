; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude FocusPoint.msg.html

(cl:defclass <FocusPoint> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:integer
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:integer
    :initform 0))
)

(cl:defclass FocusPoint (<FocusPoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FocusPoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FocusPoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<FocusPoint> is deprecated: use kortex_driver-msg:FocusPoint instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <FocusPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:x-val is deprecated.  Use kortex_driver-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <FocusPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:y-val is deprecated.  Use kortex_driver-msg:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FocusPoint>) ostream)
  "Serializes a message object of type '<FocusPoint>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'y)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FocusPoint>) istream)
  "Deserializes a message object of type '<FocusPoint>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'y)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FocusPoint>)))
  "Returns string type for a message object of type '<FocusPoint>"
  "kortex_driver/FocusPoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FocusPoint)))
  "Returns string type for a message object of type 'FocusPoint"
  "kortex_driver/FocusPoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FocusPoint>)))
  "Returns md5sum for a message object of type '<FocusPoint>"
  "64be90712af6ea79ae6f103da824ffcf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FocusPoint)))
  "Returns md5sum for a message object of type 'FocusPoint"
  "64be90712af6ea79ae6f103da824ffcf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FocusPoint>)))
  "Returns full string definition for message of type '<FocusPoint>"
  (cl:format cl:nil "~%uint32 x~%uint32 y~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FocusPoint)))
  "Returns full string definition for message of type 'FocusPoint"
  (cl:format cl:nil "~%uint32 x~%uint32 y~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FocusPoint>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FocusPoint>))
  "Converts a ROS message object to a list"
  (cl:list 'FocusPoint
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
