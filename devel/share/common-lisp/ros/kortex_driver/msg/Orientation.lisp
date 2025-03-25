; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude Orientation.msg.html

(cl:defclass <Orientation> (roslisp-msg-protocol:ros-message)
  ((theta_x
    :reader theta_x
    :initarg :theta_x
    :type cl:float
    :initform 0.0)
   (theta_y
    :reader theta_y
    :initarg :theta_y
    :type cl:float
    :initform 0.0)
   (theta_z
    :reader theta_z
    :initarg :theta_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass Orientation (<Orientation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Orientation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Orientation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<Orientation> is deprecated: use kortex_driver-msg:Orientation instead.")))

(cl:ensure-generic-function 'theta_x-val :lambda-list '(m))
(cl:defmethod theta_x-val ((m <Orientation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:theta_x-val is deprecated.  Use kortex_driver-msg:theta_x instead.")
  (theta_x m))

(cl:ensure-generic-function 'theta_y-val :lambda-list '(m))
(cl:defmethod theta_y-val ((m <Orientation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:theta_y-val is deprecated.  Use kortex_driver-msg:theta_y instead.")
  (theta_y m))

(cl:ensure-generic-function 'theta_z-val :lambda-list '(m))
(cl:defmethod theta_z-val ((m <Orientation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:theta_z-val is deprecated.  Use kortex_driver-msg:theta_z instead.")
  (theta_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Orientation>) ostream)
  "Serializes a message object of type '<Orientation>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Orientation>) istream)
  "Deserializes a message object of type '<Orientation>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta_z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Orientation>)))
  "Returns string type for a message object of type '<Orientation>"
  "kortex_driver/Orientation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Orientation)))
  "Returns string type for a message object of type 'Orientation"
  "kortex_driver/Orientation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Orientation>)))
  "Returns md5sum for a message object of type '<Orientation>"
  "2fb1d2b19db735698f3a1ad8c8e9d45b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Orientation)))
  "Returns md5sum for a message object of type 'Orientation"
  "2fb1d2b19db735698f3a1ad8c8e9d45b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Orientation>)))
  "Returns full string definition for message of type '<Orientation>"
  (cl:format cl:nil "~%float32 theta_x~%float32 theta_y~%float32 theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Orientation)))
  "Returns full string definition for message of type 'Orientation"
  (cl:format cl:nil "~%float32 theta_x~%float32 theta_y~%float32 theta_z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Orientation>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Orientation>))
  "Converts a ROS message object to a list"
  (cl:list 'Orientation
    (cl:cons ':theta_x (theta_x msg))
    (cl:cons ':theta_y (theta_y msg))
    (cl:cons ':theta_z (theta_z msg))
))
