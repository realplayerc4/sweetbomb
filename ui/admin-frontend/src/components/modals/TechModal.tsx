import { Modal } from 'antd';
import type { ModalProps } from 'antd';

interface TechModalProps extends Omit<ModalProps, 'className'> {
  title: string;
}

export function TechModal({ title, children, width = 560, ...rest }: TechModalProps) {
  return (
    <Modal
      title={<span className="text-cyan-400 font-bold text-[14px] tracking-wider">{title}</span>}
      width={width}
      centered
      destroyOnHidden
      styles={{
        root: {
          background: '#0d1b36',
        },
        header: {
          background: 'transparent',
          borderBottom: '1px solid rgba(0,212,255,0.12)',
        },
        body: {
          background: 'transparent',
        },
        footer: {
          background: 'transparent',
          borderTop: '1px solid rgba(0,212,255,0.12)',
        },
        mask: {
          background: 'rgba(0,0,0,0.6)',
        },
      }}
      {...rest}
    >
      {children}
    </Modal>
  );
}
