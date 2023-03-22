import {ImageBackground, View} from 'react-native';
import styled from 'styled-components/native';

const LandingScreen = () => {
  return (
    <Container source={require('../assets/images/launch_screen_large.png')}>
      <UpperView>
        <Title> 새로운 여정 경험을 선사하는 </Title>
        <Title> 자율주행 서비스, 카밍 </Title>
      </UpperView>
      <LowerView>
        <BtnView>
          <LandingScreenBtn title="로그인" />
          <LandingScreenBtn title="회원가입" />
        </BtnView>
      </LowerView>
    </Container>
  );
};

const Container = styled(ImageBackground)`
  flex: 1;
  align-items: center;
  justify-content: center;
  background-color: white;
`;

const Title = styled.Text`
  font-family: 'SeoulNamsanB';
  font-size: 30px;
  color: white;
  text-align: center;
  text-shadow: 3px 3px 20px gray;
  margin-top: 10px;
`;

const UpperView = styled.View`
  flex: 1;
  align-items: center;
  justify-content: center;
`

const LowerView = styled.View`
  display: flex;
  flex: 1;
  align-items: center;
  justify-content: center;
`;

const BtnView = styled.View`
  display: flex;
  flex-direction: row;
`;

const LandingScreenBtn = styled.Button`
  // background-color: blue;
  // padding: 10px;
  // border-radius: 5px;
`;

export default LandingScreen;
