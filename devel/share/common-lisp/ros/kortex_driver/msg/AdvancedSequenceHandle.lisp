; Auto-generated. Do not edit!


(cl:in-package kortex_driver-msg)


;//! \htmlinclude AdvancedSequenceHandle.msg.html

(cl:defclass <AdvancedSequenceHandle> (roslisp-msg-protocol:ros-message)
  ((handle
    :reader handle
    :initarg :handle
    :type kortex_driver-msg:SequenceHandle
    :initform (cl:make-instance 'kortex_driver-msg:SequenceHandle))
   (in_loop
    :reader in_loop
    :initarg :in_loop
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass AdvancedSequenceHandle (<AdvancedSequenceHandle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AdvancedSequenceHandle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AdvancedSequenceHandle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kortex_driver-msg:<AdvancedSequenceHandle> is deprecated: use kortex_driver-msg:AdvancedSequenceHandle instead.")))

(cl:ensure-generic-function 'handle-val :lambda-list '(m))
(cl:defmethod handle-val ((m <AdvancedSequenceHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:handle-val is deprecated.  Use kortex_driver-msg:handle instead.")
  (handle m))

(cl:ensure-generic-function 'in_loop-val :lambda-list '(m))
(cl:defmethod in_loop-val ((m <AdvancedSequenceHandle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kortex_driver-msg:in_loop-val is deprecated.  Use kortex_driver-msg:in_loop instead.")
  (in_loop m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AdvancedSequenceHandle>) ostream)
  "Serializes a message object of type '<AdvancedSequenceHandle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'handle) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'in_loop) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AdvancedSequenceHandle>) istream)
  "Deserializes a message object of type '<AdvancedSequenceHandle>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'handle) istream)
    (cl:setf (cl:slot-value msg 'in_loop) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AdvancedSequenceHandle>)))
  "Returns string type for a message object of type '<AdvancedSequenceHandle>"
  "kortex_driver/AdvancedSequenceHandle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AdvancedSequenceHandle)))
  "Returns string type for a message object of type 'AdvancedSequenceHandle"
  "kortex_driver/AdvancedSequenceHandle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AdvancedSequenceHandle>)))
  "Returns md5sum for a message object of type '<AdvancedSequenceHandle>"
  "7661973c2e6febb748e9471014616c3e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AdvancedSequenceHandle)))
  "Returns md5sum for a message object of type 'AdvancedSequenceHandle"
  "7661973c2e6febb748e9471014616c3e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AdvancedSequenceHandle>)))
  "Returns full string definition for message of type '<AdvancedSequenceHandle>"
  (cl:format cl:nil "~%SequenceHandle handle~%bool in_loop~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AdvancedSequenceHandle)))
  "Returns full string definition for message of type 'AdvancedSequenceHandle"
  (cl:format cl:nil "~%SequenceHandle handle~%bool in_loop~%================================================================================~%MSG: kortex_driver/SequenceHandle~%~%uint32 identifier~%uint32 permission~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AdvancedSequenceHandle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'handle))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AdvancedSequenceHandle>))
  "Converts a ROS message object to a list"
  (cl:list 'AdvancedSequenceHandle
    (cl:cons ':handle (handle msg))
    (cl:cons ':in_loop (in_loop msg))
))
