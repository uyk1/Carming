import { Picker } from '@react-native-picker/picker';
import { Formik, Form, Field, ErrorMessage } from 'formik';
import { View, Text, TextInput, Button, KeyboardAvoidingView, ScrollView } from 'react-native';
import styled from 'styled-components';
import * as Yup from 'yup';

export type RegistFormValues = {
  phone: string;
  name: string;
  nickname: string;
  password: string;
  passwordConfirm: string;
  gender: 'male' | 'female';
  birthDate: string;
  cardNumber: string;
  ccv: string;
  cardExp: string;
};

const initialValues: RegistFormValues = {
  phone: '',
  name: '',
  nickname: '',
  password: '',
  passwordConfirm: '',
  gender: 'male',
  birthDate: '',
  cardNumber: '',
  ccv: '',
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
  passwordConfirm: Yup.string()
    .oneOf([Yup.ref('password'), ''], '비밀번호가 일치하지 않습니다.'),
    // .required('필수 항목입니다.'),
  gender: Yup.string()
    .oneOf(['male', 'female'], '남성 또는 여성을 선택해주세요.'),
    // .required('필수 항목입니다.'),
  birthDate: Yup.string()
    // .required('필수 항목입니다.'),
    .matches(/^[0-9]{4}\/[0-9]{2}\/[0-9]{2}$/, 'YYYY/MM/DD 형식으로 입력해주세요.'),
  cardNumber: Yup.string()
    // .required('필수 항목입니다.')
    .matches(/^[0-9]+$/, '숫자만 입력 가능합니다.')
    .min(16, '카드번호는 16자리여야 합니다.'),
  ccv: Yup.string()
    // .required('필수 항목입니다.')
    .matches(/^[0-9]+$/, '숫자만 입력 가능합니다.')
    .min(3, 'CCV/CVC는 3자리여야 합니다.')
    .max(3, 'CCV/CVC는 3자리여야 합니다.'),
  cardExp: Yup.string()
    // .required('필수 항목입니다.')
    .matches(/^[0-9]{2}\/[0-9]{2}$/, 'MM/YY 형식으로 입력해주세요.'),
});

export type RegistFormProps = {
  onSubmit: (values: RegistFormValues) => void;
};

const RegistForm: React.FC<RegistFormProps> = ({ onSubmit }) => {
  const handleSubmit = (values: RegistFormValues) => {
    onSubmit(values);
  };

  return (
    <RegistFormView>
      <Formik
        initialValues={initialValues}
        validationSchema={RegistSchema}
        onSubmit={handleSubmit}
      >
        {({ handleChange, handleBlur, handleSubmit, values, errors }) => (
          <View>
            <View>
              <RegistFormText>전화번호</RegistFormText>
              <TextInput
                value={values.phone}
                placeholder='전화번호를 입력해주세요'
                placeholderTextColor="#747273"
                onChangeText={handleChange('phone')}
                onBlur={handleBlur('phone')}
                keyboardType='numeric'
                style={{ color: '#747273' }}
              />
              {errors.phone && <RegistErrorMessage>{errors.phone}</RegistErrorMessage>}
            </View>
            
            <View>
              <RegistFormText>이름</RegistFormText>
              <TextInput
                value={values.name}              
                placeholder='이름을 입력해주세요'
                placeholderTextColor="#747273"
                onChangeText={handleChange('name')}
                onBlur={handleBlur('name')}
              />
              {errors.name && <RegistErrorMessage>{errors.name}</RegistErrorMessage>}
            </View>
            
            <View>
              <RegistFormText>닉네임</RegistFormText>
              <TextInput
                value={values.nickname}              
                placeholder='사용할 닉네임을 입력해주세요'
                placeholderTextColor="#747273"
                onChangeText={handleChange('nickname')}
                onBlur={handleBlur('nickname')}
              />
              {errors.nickname && <RegistErrorMessage>{errors.nickname}</RegistErrorMessage>}
            </View>
            
            <View>
              <RegistFormText>비밀번호</RegistFormText>
              <TextInput
                value={values.password}              
                placeholder='비밀번호를 입력해주세요'
                placeholderTextColor="#747273"
                onChangeText={handleChange('password')}
                onBlur={handleBlur('password')}
                secureTextEntry
              />
              {errors.password && <RegistErrorMessage>{errors.password}</RegistErrorMessage>}
            </View>

            <View>
              <RegistFormText>비밀번호 확인</RegistFormText>
              <TextInput
                value={values.passwordConfirm}              
                placeholder='비밀번호를 확인해주세요'
                placeholderTextColor="#747273"
                onChangeText={handleChange('passwordConfirm')}
                onBlur={handleBlur('passwordConfirm')}
                secureTextEntry
              />
              {errors.passwordConfirm && <RegistErrorMessage>{errors.passwordConfirm}</RegistErrorMessage>}
            </View>
            
            <View>
              <RegistFormText>성별</RegistFormText>
              <View
                style={{
                  marginTop: 2,
                  marginBottom: 5,
                  backgroundColor: 'rgba(255,255,255,0.5)',
                  borderRadius: 10,
                }}
              >
                <Picker
                  selectedValue={values.gender}
                  onValueChange={handleChange('gender')}
                  style={{ color: '#747273' }}
                >
                  <Picker.Item style={{fontSize:13}} label="남성" value="male" />
                  <Picker.Item style={{fontSize:13}} label="여성" value="female" />
                </Picker>
              </View>
              {errors.gender && <RegistErrorMessage>{errors.gender}</RegistErrorMessage>}
            </View>
            
            <View>
              <RegistFormText>생년월일</RegistFormText>
              <TextInput
                value={values.birthDate}              
                placeholder='생년월일을 입력해주세요'
                placeholderTextColor="#747273"
                onChangeText={handleChange('birthDate')}
                onBlur={handleBlur('birthDate')}
              />
              {errors.birthDate && <RegistErrorMessage>{errors.birthDate}</RegistErrorMessage>}
            </View>
            
            <View>
              <RegistFormText>카드 번호</RegistFormText>
              <TextInput
                value={values.cardNumber}              
                placeholder='카드 번호를 입력해주세요'
                placeholderTextColor="#747273"
                onChangeText={handleChange('cardNumber')}
                onBlur={handleBlur('cardNumber')}
              />
              {errors.cardNumber && <RegistErrorMessage>{errors.cardNumber}</RegistErrorMessage>}
            </View>
            
            <View>
              <RegistFormText>CCV/CVC</RegistFormText>
              <TextInput
                value={values.ccv}              
                placeholder='카드의 CCV/CVC를 입력해주세요'
                placeholderTextColor="#747273"
                onChangeText={handleChange('ccv')}
                onBlur={handleBlur('ccv')}
              />
              {errors.ccv && <RegistErrorMessage>{errors.ccv}</RegistErrorMessage>}
            </View>
            
            <View>
              <RegistFormText>카드 유효기간</RegistFormText>
              <TextInput
                value={values.cardExp}              
                placeholder='카드 유효기간을 입력해주세요'
                placeholderTextColor="#747273"
                onChangeText={handleChange('cardExp')}
                onBlur={handleBlur('cardExp')}
              />
              {errors.cardExp && <RegistErrorMessage>{errors.cardExp}</RegistErrorMessage>}
            </View>

            <View style={{marginTop:'5%'}}>
              <Button title="회원가입" color={'#FFBDC1'}  onPress={handleSubmit} />
            </View>
          </View>
        )}
      </Formik>
    </RegistFormView>
  );
};

const RegistFormView = styled(View)`
  padding: 10%;
  background-color: rgba(255,255,255,0.5);
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

export default RegistForm;