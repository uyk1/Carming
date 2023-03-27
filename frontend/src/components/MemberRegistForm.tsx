import React, {useState} from 'react';
import {Picker} from '@react-native-picker/picker';
import {Formik, Form, Field, ErrorMessage} from 'formik';
import {
  View,
  Text,
  TextInput,
  Button,
  KeyboardAvoidingView,
  ScrollView,
} from 'react-native';
import styled from 'styled-components';
import * as Yup from 'yup';
import CommonIcon from './CommonIcon';
import MaterialIcon from 'react-native-vector-icons/MaterialCommunityIcons';
import CustomButton from './CustomButton';

export type RegistFormValues = {
  phone: string;
  name: string;
  nickname: string;
  password: string;
  passwordConfirm: string;
  gender: 'MAIL' | 'FEMAIL';
  birthDate: string;
  cardNumber: string;
  cvv: string;
  cardExp: string;
};

const initialValues: RegistFormValues = {
  phone: '',
  name: '',
  nickname: '',
  password: '',
  passwordConfirm: '',
  gender: 'MAIL',
  birthDate: '',
  cardNumber: '',
  cvv: '',
  cardExp: '',
};

const RegistSchema = Yup.object().shape({
  phone: Yup.string(),
  // .required('필수 항목입니다.'),
  name: Yup.string(),
  // .required('필수 항목입니다.'),
  nickname: Yup.string(),
  // .required('필수 항목입니다.'),
  password: Yup.string()
    // .required('필수 항목입니다.')
    .min(6, '비밀번호는 최소 6자 이상이어야 합니다.'),
  passwordConfirm: Yup.string().oneOf(
    [Yup.ref('password'), ''],
    '비밀번호가 일치하지 않습니다.',
  ),
  // .required('필수 항목입니다.'),
  gender: Yup.string().oneOf(
    ['MAIL', 'FEMAIL'],
    '남성 또는 여성을 선택해주세요.',
  ),
  // .required('필수 항목입니다.'),
  birthDate: Yup.string()
    // .required('필수 항목입니다.'),
    .matches(
      /^[0-9]{4}\/[0-9]{2}\/[0-9]{2}$/,
      'YYYY/MM/DD 형식으로 입력해주세요.',
    ),
  cardNumber: Yup.string()
    // .required('필수 항목입니다.')
    .matches(
      /^[0-9]{4}\/[0-9]{4}\/[0-9]{4}\/[0-9]{4}$/,
      'XXXX/XXXX/XXXX/XXXX형식으로 입력해주세요.',
    )
    .max(19, '카드번호는 16자리여야 합니다.'),
  cvv: Yup.string()
    // .required('필수 항목입니다.')
    .matches(/^[0-9]+$/, '숫자만 입력 가능합니다.')
    .min(3, 'CVV/CVC는 3자리여야 합니다.')
    .max(3, 'CVV/CVC는 3자리여야 합니다.'),
  cardExp: Yup.string()
    // .required('필수 항목입니다.')
    .matches(/^[0-9]{2}\/[0-9]{2}$/, 'MM/YY 형식으로 입력해주세요.'),
});

export type RegistFormProps = {
  onSubmit: (values: RegistFormValues) => void;
};

const RegistForm: React.FC<RegistFormProps> = ({onSubmit}) => {
  const handleSubmit = (values: RegistFormValues) => {
    onSubmit(values);
  };

  return (
    <RegistFormView>
      <Formik
        initialValues={initialValues}
        validationSchema={RegistSchema}
        onSubmit={handleSubmit}>
        {({handleChange, handleBlur, handleSubmit, values, errors}) => (
          <View>
            <View>
              <InputView>
                <CommonIcon
                  iconName="call-outline"
                  iconStyle={{marginRight: 5}}
                />
                <TextInput
                  value={values.phone}
                  placeholder="전화번호를 입력해주세요"
                  placeholderTextColor="#747273"
                  onChangeText={handleChange('phone')}
                  onBlur={handleBlur('phone')}
                  keyboardType="numeric"
                  style={{color: '#747273'}}
                />
                <CustomButton
                  text="인증하기"
                  textStyle={{color: 'white', fontSize: 12}}
                  buttonStyle={{
                    backgroundColor: '#FFBDC1',
                    padding: 7,
                    borderRadius: 3,
                  }}
                />
              </InputView>
              {errors.phone && (
                <RegistErrorMessage>{errors.phone}</RegistErrorMessage>
              )}
            </View>

            <View>
              <InputView>
                <CommonIcon
                  iconName="person-circle-outline"
                  iconStyle={{marginRight: 5}}
                />
                <TextInput
                  value={values.name}
                  placeholder="이름을 입력해주세요"
                  placeholderTextColor="#747273"
                  onChangeText={handleChange('name')}
                  onBlur={handleBlur('name')}
                />
              </InputView>
              {errors.name && (
                <RegistErrorMessage>{errors.name}</RegistErrorMessage>
              )}
            </View>

            <View>
              <InputView>
                <CommonIcon
                  iconName="person-outline"
                  iconStyle={{marginRight: 5}}
                />
                <TextInput
                  value={values.nickname}
                  placeholder="사용할 닉네임을 입력해주세요"
                  placeholderTextColor="#747273"
                  onChangeText={handleChange('nickname')}
                  onBlur={handleBlur('nickname')}
                />
              </InputView>
              {errors.nickname && (
                <RegistErrorMessage>{errors.nickname}</RegistErrorMessage>
              )}
            </View>

            <View>
              <InputView>
                <CommonIcon
                  iconName="lock-closed-outline"
                  iconStyle={{marginRight: 5}}
                />
                <TextInput
                  value={values.password}
                  placeholder="비밀번호를 입력해주세요"
                  placeholderTextColor="#747273"
                  onChangeText={handleChange('password')}
                  onBlur={handleBlur('password')}
                  secureTextEntry
                />
              </InputView>
              {errors.password && (
                <RegistErrorMessage>{errors.password}</RegistErrorMessage>
              )}
            </View>

            <View>
              <InputView>
                <CommonIcon
                  iconName="lock-open-outline"
                  iconStyle={{marginRight: 5}}
                />
                <TextInput
                  value={values.passwordConfirm}
                  placeholder="비밀번호를 확인해주세요"
                  placeholderTextColor="#747273"
                  onChangeText={handleChange('passwordConfirm')}
                  onBlur={handleBlur('passwordConfirm')}
                  secureTextEntry
                />
              </InputView>
              {errors.passwordConfirm && (
                <RegistErrorMessage>
                  {errors.passwordConfirm}
                </RegistErrorMessage>
              )}
            </View>

            <InputView>
              {/* <InputView> */}
              <MaterialIcon
                name="gender-male-female"
                size={20}
                color="#747273"
              />
              {/* <RegistFormText>성별</RegistFormText> */}
              <View
                style={{
                  width: '40%',
                  marginTop: 2,
                  marginBottom: 5,
                  // backgroundColor: 'rgba(255,255,255,0.3)',
                  borderRadius: 10,
                }}>
                <Picker
                  selectedValue={values.gender}
                  onValueChange={handleChange('gender')}
                  style={{color: '#747273'}}>
                  <Picker.Item
                    style={{fontSize: 13}}
                    label="남성"
                    value="MAIL"
                  />
                  <Picker.Item
                    style={{fontSize: 13}}
                    label="여성"
                    value="FEMAIL"
                  />
                </Picker>
              </View>
              {/* </InputView> */}
              {errors.gender && (
                <RegistErrorMessage>{errors.gender}</RegistErrorMessage>
              )}
            </InputView>

            <View>
              <InputView>
                <CommonIcon
                  iconName="calendar-outline"
                  iconStyle={{marginRight: 5}}
                />
                <TextInput
                  value={values.birthDate}
                  placeholder="생년월일을 입력해주세요"
                  placeholderTextColor="#747273"
                  onChangeText={handleChange('birthDate')}
                  onBlur={handleBlur('birthDate')}
                />
              </InputView>
              {errors.birthDate && (
                <RegistErrorMessage>{errors.birthDate}</RegistErrorMessage>
              )}
            </View>

            <View>
              <InputView>
                <CommonIcon
                  iconName="card-outline"
                  iconStyle={{marginRight: 5}}
                />
                <TextInput
                  value={values.cardNumber}
                  placeholder="카드 번호를 입력해주세요"
                  placeholderTextColor="#747273"
                  onChangeText={handleChange('cardNumber')}
                  onBlur={handleBlur('cardNumber')}
                />
              </InputView>
              {errors.cardNumber && (
                <RegistErrorMessage>{errors.cardNumber}</RegistErrorMessage>
              )}
            </View>

            <View>
              <InputView>
                {/* <CommonIcon
                  iconName="add-outline"
                  iconStyle={{marginRight: 5}}
                /> */}
                <RegistFormText style={{marginRight: 5}}>
                  CVV/CVC
                </RegistFormText>
                <TextInput
                  value={values.cvv}
                  placeholder="CVV/CVC를 입력해주세요"
                  placeholderTextColor="#747273"
                  onChangeText={handleChange('cvv')}
                  onBlur={handleBlur('cvv')}
                />
              </InputView>
              {errors.cvv && (
                <RegistErrorMessage>{errors.cvv}</RegistErrorMessage>
              )}
            </View>

            <View>
              <InputView>
                {/* <CommonIcon
                  iconName="add-outline"
                  iconStyle={{marginRight: 5}}
                /> */}
                <RegistFormText style={{marginRight: 5}}>
                  유효기간
                </RegistFormText>
                <TextInput
                  value={values.cardExp}
                  placeholder="유효기간을 입력해주세요"
                  placeholderTextColor="#747273"
                  onChangeText={handleChange('cardExp')}
                  onBlur={handleBlur('cardExp')}
                />
              </InputView>
              {errors.cardExp && (
                <RegistErrorMessage>{errors.cardExp}</RegistErrorMessage>
              )}
            </View>

            <View style={{marginTop: '5%'}}>
              <Button
                title="회원가입"
                color={'#FFBDC1'}
                onPress={handleSubmit}
              />
            </View>
          </View>
        )}
      </Formik>
    </RegistFormView>
  );
};

const RegistFormView = styled(View)`
  padding: 10%;
  background-color: rgba(255, 255, 255, 0.5);
  border-radius: 10;
`;

const RegistFormText = styled(Text)`
  // font-family: SeoulNamsanM,
  font-size: 12px;
  color: grey;
`;

const RegistErrorMessage = styled(Text)`
  color: red;
  font-size: 10px;
  margin-left: 5px;
  margin-bottom: 3px;
`;

const InputView = styled(View)`
  flex-direction: row;
  align-items: center;
`;

export default RegistForm;
