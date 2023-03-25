import {StyleSheet, View, ImageBackground, Image} from 'react-native';
import LoginForm from '../components/LoginForm';
import styled from 'styled-components/native';
import BackButton from '../components/BackButton';

const LoginScreen = () => {
  return (
      <Container source={require('../assets/images/login_screen.png')}>
        <View style={styles.container}>
          <View style={{ flexDirection: 'row', marginBottom:'10%' }}>
            <BackButton />
          </View>
          <View style={styles.logoContainer}>
            <Image source={require('../assets/images/logo_white.png')}/>
          </View>
          <LoginForm />
        </View>
      </Container>
  );
};

const Container = styled(ImageBackground)`
  flex: 1;
  padding-top: 10%;
  align-items: center;
  // justify-content: center;
  background-color: white;
`;

const styles = StyleSheet.create({
  container: {
    width: '80%',
    justifyContent: 'center',
    paddingHorizontal: 10,
  },
  logoContainer: {
    justifyContent: 'center',
    alignItems: 'center',
    alignSelf: 'center',
    marginBottom: '20%',
  }
});

export default LoginScreen;
