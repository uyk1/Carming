import {Member} from './Member';

export type LoginResponsePayload = {
  token: string;
  member: Member;
};
