# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: header.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='header.proto',
  package='phoenix.msg.common',
  syntax='proto2',
  serialized_pb=_b('\n\x0cheader.proto\x12\x12phoenix.msg.common\"}\n\x06Header\x12\x14\n\x05valid\x18\x01 \x01(\x08:\x05\x66\x61lse\x12\x13\n\x08sequence\x18\x02 \x01(\r:\x01\x30\x12\x14\n\ttimestamp\x18\x03 \x01(\x03:\x01\x30\x12\x18\n\rsrc_module_id\x18\x04 \x01(\r:\x01\x30\x12\x18\n\rdst_module_id\x18\x05 \x01(\r:\x01\x30')
)




_HEADER = _descriptor.Descriptor(
  name='Header',
  full_name='phoenix.msg.common.Header',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='valid', full_name='phoenix.msg.common.Header.valid', index=0,
      number=1, type=8, cpp_type=7, label=1,
      has_default_value=True, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='sequence', full_name='phoenix.msg.common.Header.sequence', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='timestamp', full_name='phoenix.msg.common.Header.timestamp', index=2,
      number=3, type=3, cpp_type=2, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='src_module_id', full_name='phoenix.msg.common.Header.src_module_id', index=3,
      number=4, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='dst_module_id', full_name='phoenix.msg.common.Header.dst_module_id', index=4,
      number=5, type=13, cpp_type=3, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=36,
  serialized_end=161,
)

DESCRIPTOR.message_types_by_name['Header'] = _HEADER
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Header = _reflection.GeneratedProtocolMessageType('Header', (_message.Message,), dict(
  DESCRIPTOR = _HEADER,
  __module__ = 'header_pb2'
  # @@protoc_insertion_point(class_scope:phoenix.msg.common.Header)
  ))
_sym_db.RegisterMessage(Header)


# @@protoc_insertion_point(module_scope)