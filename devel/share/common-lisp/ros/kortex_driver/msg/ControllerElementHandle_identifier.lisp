; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude ControllerElementHandle_identifier.msg.html

(cl:defclass <ControllerElementHandle_identifier> (roslisp-msg-protocol:ros-message)
  ((button
    :reader button
    :initarg :button
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (axis
    :reader axis
    :initarg :axis
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass ControllerElementHandle_identifier (<ControllerElementHandle_identifier>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerElementHandle_identifier>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerElementHandle_identifier)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<ControllerElementHandle_identifier> is deprecated: use kortex_driver-msg:ControllerElementHandle_identifier instead.")))

(cl:ensure-generic-function 'button-val :lambda-list '(m))
(cl:defmethod button-val ((m <ControllerElementHandle_identifier>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:button-val is deprecated.  Use kortex_driver-msg:button instead.")
  (button m))

(cl:ensure-generic-function 'axis-val :lambda-list '(m))
(cl:defmethod axis-val ((m <ControllerElementHandle_identifier>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:axis-val is deprecated.  Use kortex_driver-msg:axis instead.")
  (axis m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerElementHandle_identifier>) ostream)
  "Serializes a message object of type '<ControllerElementHandle_identifier>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'button))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'button))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'axis))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'axis))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerElementHandle_identifier>) istream)
  "Deserializes a message object of type '<ControllerElementHandle_identifier>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'button) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'button)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'axis) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'axis)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerElementHandle_identifier>)))
  "Returns string type for a message object of type '<ControllerElementHandle_identifier>"
  "kortex_driver/ControllerElementHandle_identifier")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerElementHandle_identifier)))
  "Returns string type for a message object of type 'ControllerElementHandle_identifier"
  "kortex_driver/ControllerElementHandle_identifier")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerElementHandle_identifier>)))
  "Returns md5sum for a message object of type '<ControllerElementHandle_identifier>"
  "41dfd5b8a1f2350cdbaae4a32c67f201")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerElementHandle_identifier)))
  "Returns md5sum for a message object of type 'ControllerElementHandle_identifier"
  "41dfd5b8a1f2350cdbaae4a32c67f201")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerElementHandle_identifier>)))
  "Returns full string definition for message of type '<ControllerElementHandle_identifier>"
  (cl:format cl:nil "~%uint32[] button~%uint32[] axis~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerElementHandle_identifier)))
  "Returns full string definition for message of type 'ControllerElementHandle_identifier"
  (cl:format cl:nil "~%uint32[] button~%uint32[] axis~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerElementHandle_identifier>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'button) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'axis) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerElementHandle_identifier>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerElementHandle_identifier
    (cl:cons ':button (button msg))
    (cl:cons ':axis (axis msg))
))
