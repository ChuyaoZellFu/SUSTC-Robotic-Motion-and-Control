; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude PartNumberRevision.msg.html

(cl:defclass <PartNumberRevision> (roslisp-msg-protocol:ros-message)
  ((part_number_revision
    :reader part_number_revision
    :initarg :part_number_revision
    :type cl:string
    :initform ""))
)

(cl:defclass PartNumberRevision (<PartNumberRevision>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PartNumberRevision>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PartNumberRevision)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<PartNumberRevision> is deprecated: use kortex_driver-msg:PartNumberRevision instead.")))

(cl:ensure-generic-function 'part_number_revision-val :lambda-list '(m))
(cl:defmethod part_number_revision-val ((m <PartNumberRevision>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:part_number_revision-val is deprecated.  Use kortex_driver-msg:part_number_revision instead.")
  (part_number_revision m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PartNumberRevision>) ostream)
  "Serializes a message object of type '<PartNumberRevision>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'part_number_revision))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'part_number_revision))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PartNumberRevision>) istream)
  "Deserializes a message object of type '<PartNumberRevision>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'part_number_revision) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'part_number_revision) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PartNumberRevision>)))
  "Returns string type for a message object of type '<PartNumberRevision>"
  "kortex_driver/PartNumberRevision")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PartNumberRevision)))
  "Returns string type for a message object of type 'PartNumberRevision"
  "kortex_driver/PartNumberRevision")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PartNumberRevision>)))
  "Returns md5sum for a message object of type '<PartNumberRevision>"
  "4621433c3d6219edd88cd50a0ae9aa72")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PartNumberRevision)))
  "Returns md5sum for a message object of type 'PartNumberRevision"
  "4621433c3d6219edd88cd50a0ae9aa72")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PartNumberRevision>)))
  "Returns full string definition for message of type '<PartNumberRevision>"
  (cl:format cl:nil "~%string part_number_revision~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PartNumberRevision)))
  "Returns full string definition for message of type 'PartNumberRevision"
  (cl:format cl:nil "~%string part_number_revision~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PartNumberRevision>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'part_number_revision))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PartNumberRevision>))
  "Converts a ROS message object to a list"
  (cl:list 'PartNumberRevision
    (cl:cons ':part_number_revision (part_number_revision msg))
))
